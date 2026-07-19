//! P3 "first light": an in-code rectangle-with-hole flows all the way through —
//! `cam-geo` offset → `cam-cldata` → `cam-post` grbl — into a real `.nc`.
//!
//! Proven three ways: no error diagnostics, a **golden `.nc`** (byte-stable), a
//! **semantic check** on the emitted G-code, and an **ASCII backplot** of the
//! cutting motions (a visual golden).

use cam_cldata::{MoveKind, Point3, Program, SpindleDir, Step};
use cam_geo::{Contour, Point};
use cam_model::{
    Comp, Document, Heights, Lead, Machine, Operation, Plunge, ProfileOp, Setup, Side, Stock, Tool,
    ToolKind,
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
        length: 30.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    };
    let outer = ProfileOp {
        clearing: cam_model::Clearing::default(),
        id: 0,
        tool: 1,
        chain: rect(10.0, 10.0, 70.0, 50.0),
        side: Side::Outside,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 4.0,
        stepdown: 2.0,
        stepover: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    };
    let hole = ProfileOp {
        clearing: cam_model::Clearing::default(),
        id: 1,
        tool: 1,
        chain: rect(35.0, 25.0, 45.0, 35.0),
        side: Side::Inside,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 4.0,
        stepdown: 2.0,
        stepover: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    };
    Document::new(Setup {
        name: "first light".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: 0.0,
            thickness: 10.0,
        },
        tools: vec![tool],
        operations: vec![Operation::Profile(outer), Operation::Profile(hole)],
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
    })
}

fn plan_and_post() -> (Program, String, Vec<cam_toolpath::Diagnostic>) {
    let doc = document();
    let (program, diags) = build_job(&doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    let opts = PostOptions {
        program_name: Some("first_light".into()),
        ..Default::default()
    };
    let nc = GrblPost.post(&program, &machine(), &opts).expect("post ok");
    (program, nc, diags)
}

#[test]
fn a_start_point_prepends_a_rapid_to_it() {
    let mut doc = document();
    doc.setup.origin = [1.0, 2.0, 0.0];
    doc.setup.start_offset = Some([0.0, 0.0, 30.0]); // origin + offset ⇒ (1,2,30)
    let (program, _) = build_job(&doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    // The very first motion is a rapid to origin + offset.
    let first_rapid = program
        .steps()
        .iter()
        .find_map(|s| match s {
            Step::Rapid { to, .. } => Some(*to),
            _ => None,
        })
        .expect("a rapid");
    assert_eq!(first_rapid, Point3::new(1.0, 2.0, 30.0));
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
        length: 30.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    };
    let op = ProfileOp {
        clearing: cam_model::Clearing::default(),
        id: 0,
        tool: 1,
        chain: rect(35.0, 25.0, 45.0, 35.0),
        side: Side::Inside,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 4.0,
        stepdown: 2.0,
        stepover: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    };
    let tools = [tool];
    let env = JobEnv {
        heights: Heights::new(5.0, 2.0, 0.0),
        tools: &tools,
        stock: None,
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
        length: 30.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    };
    let op = ProfileOp {
        clearing: cam_model::Clearing::default(),
        id: 0,
        tool: 1,
        chain: rect(10.0, 10.0, 70.0, 50.0),
        side: Side::Outside,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 4.0,
        stepdown: 2.0,
        stepover: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    };
    let tools = [tool];
    let env = JobEnv {
        heights: Heights::new(5.0, 2.0, 0.0),
        tools: &tools,
        stock: None,
    };
    let cancel = CancelToken::new();
    cancel.cancel();
    let result = ProfileStrategy::new(op).compute(&env, &cancel);
    assert!(result.cancelled, "should report cancellation");
    assert!(result.program.is_empty(), "no motions when cancelled");
}

#[test]
fn lead_overlap_recuts_past_the_start() {
    // No leads, but a 2 mm closure overlap: after the loop closes back at the
    // plunge point, the tool keeps *cutting* 2 mm along the contour before it
    // retracts — re-machining the entry/exit witness. On-line comp (Side::On)
    // cuts the sharp chain itself, so the geometry is exact: start (10,10),
    // first edge +X, overlap point (12,10).
    let tool = Tool {
        number: 1,
        diameter: 6.0,
        length: 30.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    };
    let overlap = 2.0;
    let op = ProfileOp {
        clearing: cam_model::Clearing::default(),
        id: 0,
        tool: 1,
        chain: rect(10.0, 10.0, 70.0, 50.0),
        side: Side::On,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 2.0,
        stepdown: 2.0,
        stepover: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: overlap,
        plunge: Plunge::Straight,
    };
    let tools = [tool];
    let env = JobEnv {
        heights: Heights::new(5.0, 2.0, 0.0),
        tools: &tools,
        stock: None,
    };
    let result = ProfileStrategy::new(op).compute(&env, &CancelToken::new());
    assert!(!result.has_errors(), "{:?}", result.diagnostics);
    let steps = result.program.steps();

    // The straight plunge lands on the start (no lead-in).
    let plunge = steps
        .iter()
        .find_map(|s| match s {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => Some(*to),
            _ => None,
        })
        .expect("a straight plunge");
    assert!((plunge.x - 10.0).abs() < 1e-6 && (plunge.y - 10.0).abs() < 1e-6);

    // The retract lifts from the overlap point, 2 mm along +X from the start.
    let retract = steps
        .iter()
        .find_map(|s| match s {
            Step::Rapid { to, tag } if tag.kind == MoveKind::Retract => Some(*to),
            _ => None,
        })
        .expect("a retract");
    assert!(
        (retract.x - 12.0).abs() < 1e-6 && (retract.y - 10.0).abs() < 1e-6,
        "retract should lift 2 mm past the start, got ({}, {})",
        retract.x,
        retract.y
    );

    // The overlap is *cut*, not rapided: the last cutting move ends where we lift.
    let last_cut = steps
        .iter()
        .rev()
        .find_map(|s| match s {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(*to),
            Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some(*end),
            _ => None,
        })
        .expect("cutting moves");
    assert!(
        (last_cut.x - 12.0).abs() < 1e-6 && (last_cut.y - 10.0).abs() < 1e-6,
        "the last cut should end at the overlap point"
    );
}

#[test]
fn offset_leaves_stock_on_the_wall() {
    // An outside profile of a 60×40 rect with a ⌀6 tool (r=3) runs at the chain
    // edge + radius; a finishing `offset` pushes the whole path that much further
    // out, leaving `offset` mm of stock on the wall for a later finishing pass.
    let run_offset = |offset: f64| {
        let op = ProfileOp {
            clearing: cam_model::Clearing::default(),
            id: 0,
            tool: 1,
            chain: rect(10.0, 10.0, 70.0, 50.0),
            side: Side::Outside,
            comp: Comp::Computed,
            offset,
            depth: 2.0,
            stepdown: 2.0,
            stepover: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            start: None,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
            plunge: Plunge::Straight,
        };
        let tools = [Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }];
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &tools,
            stock: None,
        };
        let result = ProfileStrategy::new(op).compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        result
            .program
            .steps()
            .iter()
            .filter_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(to.x),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some(end.x),
                _ => None,
            })
            .fold(f64::MIN, f64::max)
    };

    let base = run_offset(0.0); // right edge 70 + radius 3
    assert!((base - 73.0).abs() < 1e-6, "path runs at edge + radius, got {base}");
    let left = run_offset(2.0); // + 2 mm allowance
    assert!(
        (left - base - 2.0).abs() < 1e-6,
        "a 2 mm offset leaves the wall 2 mm proud, got {left}"
    );
}

fn plunge_count(prog: &Program) -> usize {
    prog.steps()
        .iter()
        .filter(|s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge))
        .count()
}

fn rough_op(side: Side, chain: Contour, stepover: f64) -> ProfileOp {
    ProfileOp {
        clearing: cam_model::Clearing::default(),
        id: 0,
        tool: 1,
        chain,
        side,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 2.0,
        stepdown: 2.0, // one level
        stepover,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    }
}

fn end_mill_tools() -> [Tool; 1] {
    [Tool {
        number: 1,
        diameter: 6.0,
        length: 30.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    }]
}

#[test]
fn inside_profile_ignores_stepover_and_warns_on_uncut_core() {
    // Radial stepover is outside-only: an inner profile is a single-pass wall
    // finish, so a set stepover is ignored (one plunge). A hole far larger than
    // the tool would leave an uncut core, so it warns (rough it with a pocket).
    let op = rough_op(Side::Inside, rect(0.0, 0.0, 40.0, 40.0), 4.0);
    let tools = end_mill_tools();
    let env = JobEnv {
        heights: Heights::new(5.0, 2.0, 0.0),
        tools: &tools,
        stock: None,
    };
    let r = ProfileStrategy::new(op).compute(&env, &CancelToken::new());
    assert!(!r.has_errors(), "{:?}", r.diagnostics);
    assert_eq!(
        plunge_count(&r.program),
        1,
        "an inner profile is a single finishing pass, not radial roughing"
    );
    assert!(
        r.diagnostics
            .iter()
            .any(|d| d.severity == cam_toolpath::Severity::Warning),
        "warns that the inner profile leaves an uncut core"
    );
}

#[test]
fn stepover_roughs_outside_frame_to_the_stock() {
    // A 40×40 part in a 60×60 stock: outside roughing clears the 10 mm frame in
    // concentric passes (part as an island in the stock). Routed through the shared
    // clearer it now stays *down* — the frame is cut in many ring segments but with
    // far fewer plunges than moves (not a plunge-per-ring as before).
    let op = rough_op(Side::Outside, rect(10.0, 10.0, 50.0, 50.0), 4.0);
    let tools = end_mill_tools();
    let env = JobEnv {
        heights: Heights::new(5.0, 2.0, 0.0),
        tools: &tools,
        stock: Some(([0.0, 0.0], [60.0, 60.0])),
    };
    let r = ProfileStrategy::new(op).compute(&env, &CancelToken::new());
    assert!(!r.has_errors(), "{:?}", r.diagnostics);
    let cuts = r
        .program
        .steps()
        .iter()
        .filter(|s| matches!(s, Step::Linear { tag, .. } | Step::Arc { tag, .. } if tag.kind == MoveKind::Cutting))
        .count();
    let plunges = plunge_count(&r.program);
    assert!(cuts > 8, "the frame is cleared in several rings, got {cuts} cutting moves");
    assert!(plunges >= 1, "at least one entry plunge");
    assert!(
        plunges * 2 < cuts,
        "stay-down: far fewer plunges than cutting moves ({plunges} vs {cuts})"
    );

    // No gouge into the part: the tool centre stays a radius outside the 40×40 part
    // [10,50]², so no cutting move — including the stay-down links between rings —
    // may pass through the part interior. Check endpoints and segment midpoints.
    let inside_part = |x: f64, y: f64| (10.5..49.5).contains(&x) && (10.5..49.5).contains(&y);
    let mut prev: Option<(f64, f64)> = None;
    for s in r.program.steps() {
        let cutting = matches!(s, Step::Linear { tag, .. } | Step::Arc { tag, .. } if tag.kind == MoveKind::Cutting);
        let (ex, ey) = match s {
            Step::Linear { to, .. } | Step::Rapid { to, .. } => (to.x, to.y),
            Step::Arc { end, .. } => (end.x, end.y),
            _ => continue,
        };
        if cutting {
            assert!(!inside_part(ex, ey), "cutting move ends inside the part at ({ex}, {ey})");
            if let Some((px, py)) = prev {
                let (mx, my) = (0.5 * (px + ex), 0.5 * (py + ey));
                assert!(!inside_part(mx, my), "a cutting link chords through the part at ({mx}, {my})");
            }
        }
        prev = Some((ex, ey));
    }
}

#[test]
fn outside_stepover_without_stock_is_a_single_pass() {
    // No stock ⇒ there is no frame to define, so roughing falls back to a single
    // finishing pass: one plunge for the single depth level.
    let op = rough_op(Side::Outside, rect(10.0, 10.0, 50.0, 50.0), 4.0);
    let tools = end_mill_tools();
    let env = JobEnv {
        heights: Heights::new(5.0, 2.0, 0.0),
        tools: &tools,
        stock: None,
    };
    let r = ProfileStrategy::new(op).compute(&env, &CancelToken::new());
    assert!(!r.has_errors(), "{:?}", r.diagnostics);
    assert_eq!(plunge_count(&r.program), 1, "falls back to one finishing pass");
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

/// A profile with an arc lead-in/out and a helix plunge posts to valid grbl G-code
/// with helical `G2/G3` moves (a Z word on an arc) — proven against the post.
#[test]
fn arc_lead_and_helix_plunge_post_to_helical_gcode() {
    let op = ProfileOp {
        clearing: cam_model::Clearing::default(),
        id: 0,
        tool: 1,
        // Sited well inside the envelope so the outward leads keep clear of x=0/y=0.
        chain: rect(40.0, 40.0, 100.0, 80.0),
        side: Side::Outside,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 4.0,
        stepdown: 2.0,
        stepover: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::Arc { radius: 3.0 },
        lead_out: Lead::Arc { radius: 3.0 },
        lead_overlap: 0.0,
        plunge: Plunge::Helix {
            radius: 2.0,
            pitch: 1.0,
        },
    };
    let tool = Tool {
        number: 1,
        diameter: 6.0,
        length: 30.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    };
    let doc = Document::new(Setup {
        name: "lead_helix".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: 0.0,
            thickness: 10.0,
        },
        tools: vec![tool],
        operations: vec![Operation::Profile(op)],
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
    });

    let (program, diags) = build_job(&doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    assert!(
        !diags
            .iter()
            .any(|d| d.severity == cam_toolpath::Severity::Error),
        "no error diagnostics: {diags:?}"
    );
    // The program carries lead-in and helical arc moves.
    assert!(
        program.steps.iter().any(|s| matches!(s, Step::Arc { .. })),
        "expected arc moves (leads + helix)"
    );

    let nc = GrblPost
        .post(&program, &machine(), &PostOptions::default())
        .expect("post ok");
    // A helical move is a G2/G3 line that also carries a Z word.
    let helical = nc
        .lines()
        .any(|l| (l.contains("G2") || l.contains("G3")) && l.contains('Z'));
    assert!(helical, "expected a helical G2/G3 with a Z word:\n{nc}");
}

// --- multi-tool operations: the tool-change ledger ---

/// A ⌀40 disc, carved 1 mm deep with a 90° V-bit (tool 1) after a ⌀6 end mill
/// (tool 2) has cleared the flat land it leaves — then a plain profile with a
/// third tool, so the planner's bookkeeping after a multi-tool fragment is
/// exercised rather than assumed.
fn carve_document(clear_tool: Option<u32>, trailing_tool: Option<u32>) -> Document {
    let vbit = Tool {
        number: 1,
        diameter: 6.0,
        length: 30.0,
        flutes: 1,
        kind: ToolKind::VBit {
            included_angle_deg: 90.0,
            tip_radius: 0.1,
        },
        ..Default::default()
    };
    let mill = Tool {
        number: 2,
        diameter: 6.0,
        length: 30.0,
        flute_length: 20.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    };
    let other = Tool {
        number: 3,
        diameter: 3.0,
        length: 30.0,
        flute_length: 20.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    };
    let carve = cam_model::CarveOp {
        id: 0,
        tool: 1,
        clear_tool,
        boundary: rect(0.0, 0.0, 40.0, 40.0),
        islands: Vec::new(),
        top: 0.0,
        depth: 1.0,
        offset: 0.0,
        ring_step: 0.5,
        feed: 300.0,
        plunge_feed: 100.0,
        stay_down: true,
        clear_stepover: 0.0,
        clear_stepdown: 0.0,
        clear_feed: 0.0,
        clear_plunge_feed: 0.0,
        clear_plunge: Plunge::Straight,
        start: None,
    };
    let mut operations = vec![Operation::Carve(carve)];
    if let Some(t) = trailing_tool {
        operations.push(Operation::Profile(ProfileOp {
            clearing: cam_model::Clearing::default(),
            id: 1,
            tool: t,
            chain: rect(0.0, 0.0, 40.0, 40.0),
            side: Side::Outside,
            comp: Comp::Computed,
            offset: 0.0,
            depth: 2.0,
            stepdown: 2.0,
            stepover: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            start: None,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
            plunge: Plunge::Straight,
        }));
    }
    Document::new(Setup {
        name: "carve".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: 0.0,
            thickness: 10.0,
        },
        tools: vec![vbit, mill, other],
        operations,
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
    })
}

fn tool_changes(program: &Program) -> Vec<u32> {
    program
        .steps()
        .iter()
        .filter_map(|s| match s {
            Step::ToolChange { tool } => Some(*tool),
            _ => None,
        })
        .collect()
}

#[test]
fn a_two_tool_operation_loads_its_first_tool_then_changes_itself() {
    // The planner must load the *clearing* tool for a Carve, not its defining tool:
    // `tools()[0]` is what has to be in the spindle when the fragment starts. The
    // change to the V-bit belongs to the strategy, which alone knows the order.
    let doc = carve_document(Some(2), None);
    let (program, diags) = build_job(&doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    assert!(
        !diags
            .iter()
            .any(|d| d.severity == cam_toolpath::Severity::Error),
        "{diags:?}"
    );
    assert_eq!(tool_changes(&program), vec![2, 1]);
}

#[test]
fn the_planner_resyncs_the_spindle_tool_after_a_multi_tool_fragment() {
    // The bug this pins: if the planner assumed the operation left `tools()[0]` in the
    // spindle, it would think tool 2 was still loaded and emit a change to it for the
    // next operation — while the machine actually holds the V-bit. The carve would be
    // followed by a profile cut with the wrong cutter.
    let doc = carve_document(Some(2), Some(2));
    let (program, _) = build_job(&doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    // Clearing tool, hand-back to the V-bit, then genuinely back to tool 2 for the
    // profile — the last change is NOT elided.
    assert_eq!(tool_changes(&program), vec![2, 1, 2]);
}

#[test]
fn a_following_operation_on_the_carving_tool_needs_no_change() {
    // The other direction: the fragment really did leave tool 1 loaded, so an operation
    // that wants tool 1 must not be given a redundant change.
    let doc = carve_document(Some(2), Some(1));
    let (program, _) = build_job(&doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    assert_eq!(tool_changes(&program), vec![2, 1]);
}

#[test]
fn a_single_tool_carve_behaves_like_every_other_operation() {
    let doc = carve_document(None, Some(2));
    let (program, _) = build_job(&doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    assert_eq!(tool_changes(&program), vec![1, 2]);
}
