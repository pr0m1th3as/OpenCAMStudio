//! Regenerate the ribbon illustrations from the actual CAM strategies.
//! Run from the workspace root: cargo run -p cam-app --example operation_hints
use std::{f64::consts::TAU, fmt::Write as _, fs, path::Path};

use cam_app::{default_machine, AppController, OpKind};
use cam_cldata::{Program, SpindleDir, Step};
use cam_geo::{Contour, Point, Polygon};
use cam_geometry3d::{AdaptiveGeometry, AdaptiveLayer, ParallelGeometry};
use cam_model::{
    CuttingHeights, Hand, Heights, Lead, Operation, StockToLeave, ThreadOp, Tool, ToolKind,
};
use cam_render::{Scene, CUT, PLUNGE};
use cam_sim::{ProfileShape, SimOptions, SimTool, ToolProfile};
use cam_toolpath::{
    AdaptiveStrategy, BoreStrategy, CancelToken, CarveStrategy, ChamferStrategy, DrillStrategy,
    EngraveStrategy, FaceStrategy, JobEnv, ParallelStrategy, PocketStrategy, ProfileStrategy,
    SpiralStrategy, Strategy, ThreadStrategy,
};

#[path = "operation_hints/visibility.rs"]
mod visibility;
use visibility::{Clipper, Occluder};

type Xy = [f64; 2];
type Xyz = [f64; 3];

const PLATE: [Xy; 8] = [
    [-21., -9.],
    [-16., -14.],
    [14., -14.],
    [21., -7.],
    [21., 9.],
    [16., 14.],
    [-16., 14.],
    [-21., 9.],
];
const FRAME: [Xy; 4] = [[-23., -15.], [23., -15.], [23., 15.], [-23., 15.]];

fn contour(points: &[Xy]) -> Contour {
    Contour::new(points.iter().map(|&[x, y]| Point::new(x, y)).collect())
}

fn circle([x, y]: Xy, radius: f64) -> Vec<Xy> {
    (0..64)
        .map(|i| {
            let a = i as f64 * TAU / 64.;
            [x + radius * a.cos(), y + radius * a.sin()]
        })
        .collect()
}

fn at(points: &[Xy], z: f64) -> Vec<Xyz> {
    points.iter().map(|&[x, y]| [x, y, z]).collect()
}

struct Face {
    outer: Vec<Xyz>,
    holes: Vec<Vec<Xyz>>,
    fill: String,
    edge: bool,
    occludes: bool,
}

#[derive(Default)]
struct Model {
    faces: Vec<Face>,
    guides: Vec<Vec<Xyz>>,
    curved_surface: Vec<[Xyz; 3]>,
    v_groove: bool,
}

impl Model {
    fn face(&mut self, outer: Vec<Xyz>, fill: &str, edge: bool) {
        self.faces.push(Face {
            outer,
            holes: vec![],
            fill: fill.into(),
            edge,
            occludes: true,
        });
    }

    fn slab(&mut self, outline: &[Xy], top: f64, bottom: f64, holes: &[(&[Xy], f64)]) {
        for &(rim, floor) in holes {
            self.face(at(rim, floor), "#adb8c4", true);
            self.walls(rim, top, floor, true);
        }
        self.walls(outline, top, bottom, false);
        self.faces.push(Face {
            outer: at(outline, top),
            holes: holes.iter().map(|(rim, _)| at(rim, top)).collect(),
            fill: "#e1e7eb".into(),
            edge: true,
            occludes: true,
        });
    }

    fn occluders(&self) -> Vec<Occluder> {
        self.faces
            .iter()
            .filter(|face| face.occludes)
            .filter_map(|face| Occluder::new(&face.outer, &face.holes))
            .chain(
                self.curved_surface
                    .iter()
                    .filter_map(|face| Occluder::new(face, &[])),
            )
            .collect()
    }

    fn walls(&mut self, outline: &[Xy], top: f64, bottom: f64, inside: bool) {
        for (&[x0, y0], &[x1, y1]) in outline.iter().zip(outline.iter().cycle().skip(1)) {
            // Draw front exterior walls and the far interior walls of cavities.
            let facing = (y1 - y0) - 1.74 * (x1 - x0);
            if (facing < 0.) == inside {
                let shade = if (y1 - y0).abs() > (x1 - x0).abs() {
                    "#8597a8"
                } else {
                    "#a5b2bf"
                };
                self.face(
                    vec![
                        [x0, y0, top],
                        [x1, y1, top],
                        [x1, y1, bottom],
                        [x0, y0, bottom],
                    ],
                    shade,
                    false,
                );
            }
        }
    }
}

fn project([x, y, z]: Xyz) -> Xyz {
    // The third axis is the cross product of the two screen axes.
    [
        0.87 * x - 0.5 * y,
        0.33 * x + 0.46 * y - z,
        0.5 * x + 0.87 * y + 0.5652 * z,
    ]
}

fn export(
    name: &str,
    mut model: Model,
    strategy: &dyn Strategy,
    env: &JobEnv<'_>,
    cuts_only: bool,
) {
    let result = strategy.compute(env, &CancelToken::new());
    assert!(!result.has_errors(), "{name}: {:?}", result.diagnostics);
    result
        .program
        .validate()
        .unwrap_or_else(|e| panic!("{name}: {e}"));
    if model.v_groove {
        model = v_groove_model(&result.program, &env.tools[0]);
    }
    let scene = Scene::from_program_without_markers(&result.program);
    // Omit travel and entry for dense backplots. The cutting trajectories are
    // always real strategy output, including the Z of each arc/helix sample.
    let mut paths: Vec<_> = scene
        .strips
        .iter()
        .filter(|strip| !cuts_only || strip.color == CUT)
        .map(|strip| {
            (
                if name == "drill" { CUT } else { strip.color },
                strip
                    .points
                    .iter()
                    .map(|p| p.map(f64::from))
                    .collect::<Vec<_>>(),
            )
        })
        .collect();
    assert!(
        paths.iter().any(|(color, _)| *color == CUT),
        "{name}: no cuts"
    );

    if name == "adaptive" {
        // View the long side of the ramp almost straight on, so the rise of
        // its top and the height between cut levels both stand out.
        let (sin, cos) = (-27_f64).to_radians().sin_cos();
        let surface = model
            .faces
            .iter_mut()
            .flat_map(|face| face.outer.iter_mut().chain(face.holes.iter_mut().flatten()));
        let guides = model.guides.iter_mut().flatten();
        let toolpath = paths.iter_mut().flat_map(|(_, points)| points);
        for p in surface.chain(guides).chain(toolpath) {
            let [x, y, z] = *p;
            *p = [cos * x - sin * y, sin * x + cos * y, z];
        }
    }

    let mut low = [f64::INFINITY; 2];
    let mut high = [f64::NEG_INFINITY; 2];
    for p in model
        .faces
        .iter()
        .flat_map(|f| &f.outer)
        .chain(paths.iter().flat_map(|(_, p)| p))
    {
        let q = project(*p);
        for i in 0..2 {
            low[i] = low[i].min(q[i]);
            high[i] = high[i].max(q[i]);
        }
    }
    let scale = (326. / (high[0] - low[0])).min(178. / (high[1] - low[1]));
    let point = |p| {
        let [x, y, _] = project(p);
        (
            180. + (x - (high[0] + low[0]) / 2.) * scale,
            102. + (y - (high[1] + low[1]) / 2.) * scale,
        )
    };
    let trace = |out: &mut String, vertices: &[Xyz]| {
        for (i, &p) in vertices.iter().enumerate() {
            let (x, y) = point(p);
            write!(out, "{}{x:.1} {y:.1}", if i == 0 { 'M' } else { 'L' }).unwrap();
        }
    };

    let mut svg =
        String::from("<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 360 204\">\n");
    svg.push_str("<g stroke-linecap=\"round\" stroke-linejoin=\"round\">\n");
    for face in &model.faces {
        svg.push_str("<path d=\"");
        trace(&mut svg, &face.outer);
        svg.push('Z');
        for hole in &face.holes {
            trace(&mut svg, hole);
            svg.push('Z');
        }
        write!(svg, "\" fill=\"{}\" fill-rule=\"evenodd\"", face.fill).unwrap();
        if face.edge {
            svg.push_str(" stroke=\"#65798a\" stroke-width=\"0.9\"");
        } else if name == "adaptive" || (name == "thread" && face.outer.len() == 3) {
            // Seal antialiasing seams between the shaded metal facets.
            write!(svg, " stroke=\"{}\" stroke-width=\"0.35\"", face.fill).unwrap();
        }
        svg.push_str("/>\n");
    }
    for guide in &model.guides {
        svg.push_str("<path d=\"");
        trace(&mut svg, guide);
        svg.push_str("Z\" fill=\"none\" stroke=\"#546d83\" stroke-width=\"1.15\"/>\n");
    }
    let occluders = model.occluders();
    let mut clipper = Clipper::default();
    // Clip each move in 3D before emitting SVG, then join adjacent visible
    // fragments to keep strokes continuous and file sizes small.
    let (cut_width, cut_outline) = match name {
        "thread" => ("1.05", "1.75"),
        "adaptive" => ("0.82", "1.25"),
        _ => ("1.55", "2.65"),
    };
    let (cut_ink, rapid_style, entry_style) = if name == "adaptive" {
        (
            "#56cf9a",
            "stroke=\"#d3b455\" stroke-width=\"0.8\" stroke-dasharray=\"3 3\"",
            "stroke=\"#ff956c\" stroke-width=\"1\"",
        )
    } else {
        (
            "#328bf1",
            "stroke=\"#c88840\" stroke-width=\"1\"
             stroke-dasharray=\"3 3\"",
            "stroke=\"#27ae90\" stroke-width=\"1.25\"",
        )
    };
    let cut_style = format!("stroke=\"{cut_ink}\" stroke-width=\"{cut_width}\"");
    for (color, style) in [
        (cam_render::RAPID, rapid_style),
        (PLUNGE, entry_style),
        (CUT, cut_style.as_str()),
    ] {
        let mut lines = String::new();
        let mut previous = None;
        for (_, points) in paths.iter().filter(|(role, _)| *role == color) {
            for segment in points.windows(2) {
                clipper.visible(segment[0], segment[1], &occluders, |a, b| {
                    if previous != Some(a) {
                        let (x, y) = point(a);
                        write!(lines, "M{x:.2} {y:.2}").unwrap();
                    }
                    let (x, y) = point(b);
                    write!(lines, "L{x:.2} {y:.2}").unwrap();
                    previous = Some(b);
                });
            }
        }
        if !lines.is_empty() {
            if color == CUT {
                writeln!(
                    svg,
                    "<path d=\"{lines}\" fill=\"none\" stroke=\"#1a395b\" stroke-width=\"{cut_outline}\"/>"
                )
                .unwrap();
            }
            writeln!(svg, "<path d=\"{lines}\" fill=\"none\" {style}/>").unwrap();
        }
    }
    svg.push_str("</g>\n</svg>\n");
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/operation-hints");
    fs::write(dir.join(format!("{name}.svg")), svg).unwrap();
}

fn main() {
    let app = AppController::new(default_machine());
    for (name, kind) in [
        ("face", OpKind::Face),
        ("profile", OpKind::Profile),
        ("pocket", OpKind::Pocket),
        ("pocket-adaptive", OpKind::Pocket),
        ("thread", OpKind::Thread),
        ("bore", OpKind::Bore),
        ("chamfer", OpKind::Chamfer),
        ("engrave", OpKind::Engrave),
        ("carve", OpKind::Carve),
        ("drill", OpKind::Drill),
        ("adaptive", OpKind::Adaptive),
        ("parallel", OpKind::Parallel),
        ("scallop", OpKind::Parallel),
        ("spiral", OpKind::Spiral),
    ] {
        illustration(&app, name, kind);
    }
}

fn illustration(app: &AppController, name: &str, kind: OpKind) {
    let v_bit = ToolKind::VBit {
        included_angle_deg: 90.,
        tip_radius: 0.,
    };
    let (kind_of_tool, diameter) = match kind {
        OpKind::Face => (ToolKind::FaceMill, 9.),
        OpKind::Thread => (ToolKind::ThreadMill { pitch: None }, 5.),
        OpKind::Chamfer => (
            ToolKind::ChamferMill {
                included_angle_deg: 90.,
                tip_diameter: 0.,
            },
            8.,
        ),
        OpKind::Engrave => (
            ToolKind::VBit {
                included_angle_deg: 130.,
                tip_radius: 0.,
            },
            6.,
        ),
        OpKind::Carve => (v_bit, 16.),
        OpKind::Drill => (
            ToolKind::Drill {
                point_angle_deg: 118.,
            },
            4.4,
        ),
        OpKind::Parallel | OpKind::Spiral => (ToolKind::BallMill, 3.),
        _ => (ToolKind::EndMill, 4.),
    };
    let tool = Tool {
        number: 1,
        kind: kind_of_tool,
        diameter,
        length: 40.,
        flute_length: 24.,
        neck_diameter: if kind == OpKind::Thread { 2. } else { 0. },
        flutes: 2,
        ..Default::default()
    };
    let mut env = JobEnv {
        heights: Heights::new(4.5, 2., 0.),
        tools: &[tool],
        stock: None,
        spindle: SpindleDir::Cw,
    };
    let mut model = Model::default();
    let strategy: Box<dyn Strategy> = match app.operation_template(kind, 1) {
        Operation::Face(mut op) => {
            op.boundary = contour(&FRAME);
            op.start_offset = 1.;
            op.depth = 1.;
            op.stepdown = 1.;
            op.overshoot = 0.6;
            model.slab(&FRAME, 0., -5., &[]);
            Box::new(FaceStrategy::new(op))
        }
        Operation::Profile(mut op) => {
            op.chain = contour(&PLATE);
            op.depth = 6.;
            op.stepdown = 3.;
            model.slab(&PLATE, 0., -6., &[]);
            Box::new(ProfileStrategy::new(op))
        }
        Operation::Pocket(mut op) => {
            let boundary = [
                [-16., -5.],
                [-12., -9.],
                [10., -9.],
                [15., -4.],
                [15., 5.],
                [11., 9.],
                [-12., 9.],
                [-16., 5.],
            ];
            let island = circle([2., 0.], 3.2);
            op.boundary = contour(&boundary);
            op.islands = vec![contour(&island)];
            op.depth = 4.;
            op.clear.stepdown = 4.;
            op.clear.lead_in = Lead::None;
            op.clear.lead_out = Lead::None;
            if name == "pocket-adaptive" {
                op.clear.adaptive_enabled = Some(true);
                op.clear.clearing.engagement = 1.6;
                op.clear.clearing.climb = true;
            }
            model.slab(&FRAME, 0., -7., &[(&boundary, -4.)]);
            model.slab(&island, 0., -4., &[]);
            Box::new(PocketStrategy::new(op))
        }
        Operation::Bore(mut op) => {
            let outline = [[-15., -13.], [15., -13.], [15., 13.], [-15., 13.]];
            let rim = circle([0., 0.], 8.5);
            op.points = vec![[0., 0.]];
            op.diameter = 17.;
            op.z_bottom = -6.;
            op.pitch = 2.;
            model.slab(&outline, 0., -8., &[(&rim, -7.)]);
            Box::new(BoreStrategy::new(op))
        }
        Operation::Thread(mut op) => {
            op.internal = false;
            op.points = vec![[0., 0.]];
            op.major_dia = 14.;
            op.pitch = 2.;
            op.z_top = -0.5;
            op.z_bottom = -8.5;
            threaded_boss(&mut model, &op);
            Box::new(ThreadStrategy::new(op))
        }
        Operation::Chamfer(mut op) => {
            let top = inset_plate(2.);
            op.chain = contour(&PLATE);
            op.width = 2.;
            op.top = 0.;
            model.walls(&PLATE, -2., -6., false);
            for i in 0..PLATE.len() {
                let j = (i + 1) % PLATE.len();
                let a = PLATE[i];
                let b = PLATE[j];
                model.face(
                    vec![
                        [a[0], a[1], -2.],
                        [b[0], b[1], -2.],
                        [top[j][0], top[j][1], 0.],
                        [top[i][0], top[i][1], 0.],
                    ],
                    "#bdc8d1",
                    true,
                );
            }
            model.face(at(&top, 0.), "#e1e7eb", true);
            Box::new(ChamferStrategy::new(op))
        }
        Operation::Engrave(mut op) => {
            let stroke = [
                [-17., -4.],
                [-13., -4.],
                [-10., 3.],
                [-7., -5.],
                [-3., 0.],
                [1., 0.],
                [5., 7.],
                [9., -5.],
                [12., 1.],
                [17., 1.],
            ];
            op.chain = contour(&stroke);
            op.closed = false;
            op.top = 0.;
            op.depth = 0.65;
            model.v_groove = true;
            Box::new(EngraveStrategy::new(op))
        }
        Operation::Carve(mut op) => {
            let star: Vec<Xy> = (0..10)
                .map(|i| {
                    let a = TAU * i as f64 / 10.;
                    let r = if i % 2 == 0 { 12. } else { 5. };
                    [1.28 * r * a.sin(), r * a.cos()]
                })
                .collect();
            op.boundary = contour(&star);
            op.top = 0.;
            op.depth = 2.7;
            op.ring_step = 0.9;
            model.v_groove = true;
            Box::new(CarveStrategy::new(op))
        }
        Operation::Drill(mut op) => {
            op.points = vec![[-13., -6.], [-11., 6.], [0., 4.], [12., 6.], [13., -6.]];
            op.depth = 7.;
            op.peck = Some(2.);
            let rims: Vec<_> = op
                .points
                .iter()
                .map(|&p| circle(p, diameter / 2.))
                .collect();
            let holes: Vec<_> = rims.iter().map(|rim| (rim.as_slice(), -6.)).collect();
            model.slab(&PLATE, 0., -8., &holes);
            Box::new(DrillStrategy::new(op))
        }
        Operation::Adaptive(mut op) => {
            let geometry = adaptive_ramp(&mut model, &mut env);
            op.params.optimal_load = 2.6;
            op.params.stepdown = 4.5;
            op.params.fine_stepdown = 2.5;
            op.params.feed = 300.;
            op.params.plunge_feed = 100.;
            Box::new(AdaptiveStrategy::new(op, StockToLeave::default(), geometry))
        }
        Operation::Parallel(mut op) => {
            env.heights = Heights::new(12., 10., 0.);
            op.params.scallop = name == "scallop";
            op.params.stepover = if op.params.scallop { 2.1 } else { 2.6 };
            let geometry = dome(&mut model);
            Box::new(ParallelStrategy::new(op, StockToLeave::default(), geometry))
        }
        Operation::Spiral(mut op) => {
            env.heights = Heights::new(12., 10., 0.);
            op.params.stepover = 2.1;
            let geometry = dome(&mut model);
            Box::new(SpiralStrategy::new(op, StockToLeave::default(), geometry))
        }
    };
    export(
        name,
        model,
        strategy.as_ref(),
        &env,
        matches!(
            kind,
            OpKind::Pocket | OpKind::Carve | OpKind::Parallel | OpKind::Spiral
        ),
    );
}

// A solid plate with a rising convex top. Adjacent machining bands expose
// three separate horizontal cutting levels along the ramp.
fn adaptive_ramp(model: &mut Model, env: &mut JobEnv<'_>) -> AdaptiveGeometry {
    let (left, right, width, height) = (-30., 30., 13., 12.);
    let bottom = -3.5;
    let rect = |x0, x1, y0, y1| [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
    let top = |x: f64| height * (1. - ((right - x) / (right - left)).powi(2));

    let stations: Vec<_> = (0..=20)
        .map(|i| left + (right - left) * i as f64 / 20.)
        .collect();
    let top_edge = |y| stations.iter().map(|&x| [x, y, top(x)]).collect::<Vec<_>>();
    let front = top_edge(width);
    let back = top_edge(-width);

    let mut front_wall = front.clone();
    front_wall.extend([[right, width, bottom], [left, width, bottom]]);
    model.face(front_wall, "#94a7b8", true);
    model.face(
        vec![
            [right, -width, height],
            [right, width, height],
            [right, width, bottom],
            [right, -width, bottom],
        ],
        "#6f8599",
        true,
    );

    for window in stations.windows(2) {
        let (a, b) = (window[0], window[1]);
        let slope = (top(b) - top(a)) / (b - a);
        let gray = (220. - 62. * slope) as u8;
        let fill = format!("#{gray:02x}{:02x}{:02x}", gray + 7, gray + 14);
        model.face(
            vec![
                [a, -width, top(a)],
                [b, -width, top(b)],
                [b, width, top(b)],
                [a, width, top(a)],
            ],
            &fill,
            false,
        );
    }
    model
        .guides
        .push(back.into_iter().chain(front.into_iter().rev()).collect());

    env.heights = Heights::new(16., 14., height);
    env.stock = Some(([left, -width], [right, width]));
    let bands: [(f64, f64); 3] = [4., 8., 11.5].map(|z| {
        let uphill_edge = right - (right - left) * (1. - z / height).sqrt();
        (z, uphill_edge)
    });
    let layers = bands
        .into_iter()
        .enumerate()
        .rev()
        .map(|(i, (z, uphill_edge))| {
            let downhill_edge = if i == 0 { left } else { bands[i - 1].1 };
            let solid = rect(uphill_edge, right, -width, width);
            let band = rect(downhill_edge, uphill_edge, -width, width);
            AdaptiveLayer {
                z,
                protected: vec![Polygon::new(contour(&solid)).unwrap()],
                face_domains: vec![Polygon::new(contour(&band)).unwrap()],
                body_domains: vec![],
            }
        })
        .collect();
    AdaptiveGeometry {
        layers,
        fingerprint: "crowned plate".into(),
        model_top: height,
        error_bound: 0.,
    }
}

fn threaded_boss(model: &mut Model, op: &ThreadOp) {
    let [cx, cy] = op.points[0];
    let top = 0.;
    let shoulder = op.z_bottom - 1.5;
    let major = op.major_dia / 2.;
    let hand = if op.hand == Hand::Right { 1. } else { -1. };
    // Match the 60-degree external profile used by ThreadStrategy. The crest
    // occupies one eighth of a pitch and the flanks slope at 60° to the axis.
    let depth = 0.613_435 * op.pitch;
    let flank = 3_f64.sqrt();
    let point = |angle: f64, z: f64| {
        let phase = (z - op.z_top) / op.pitch - hand * angle / TAU;
        let distance = (phase - phase.round()).abs() * op.pitch;
        let groove = (flank * (7. * op.pitch / 16. - distance)).clamp(0., depth);
        // Ease the groove out into the unthreaded lands at each end.
        let runout = ((z - op.z_bottom).min(op.z_top - z) / (op.pitch / 4.)).clamp(0., 1.);
        let radius = major - runout * groove;
        [cx + radius * angle.cos(), cy + radius * angle.sin(), z]
    };

    model.slab(&circle([cx, cy], 12.), shoulder, shoulder - 3., &[]);
    let angular_steps = 72;
    let axial_steps = ((top - shoulder) / op.pitch * 14.).ceil() as usize;
    let z = |i| shoulder + (top - shoulder) * i as f64 / axial_steps as f64;
    let angle = |i| i as f64 * TAU / angular_steps as f64;
    let mut facets = Vec::new();
    for i in 0..angular_steps {
        for j in 0..axial_steps {
            let a = point(angle(i), z(j));
            let b = point(angle(i + 1), z(j));
            let c = point(angle(i + 1), z(j + 1));
            let d = point(angle(i), z(j + 1));
            for vertices in [[a, b, c], [a, c, d]] {
                if let Some((sort_depth, light)) = thread_facet(vertices) {
                    let recess = vertices
                        .iter()
                        .map(|&[x, y, _]| major - (x - cx).hypot(y - cy))
                        .sum::<f64>()
                        / (3. * depth);
                    let shade = (156. + 50. * light - 52. * recess).clamp(78., 208.) as u8;
                    facets.push((sort_depth, shade, vertices));
                }
            }
        }
    }
    facets.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
    for (_, shade, vertices) in facets {
        model.face(
            vertices.to_vec(),
            &format!("#{shade:02x}{:02x}{:02x}", shade + 8, shade + 16),
            false,
        );
    }
    model.face(at(&circle([cx, cy], major), top), "#e1e7eb", true);
}

// Cull the back of the solid and shade the flanks by their surface normals.
// The same triangles are used for drawing and toolpath occlusion.
fn thread_facet([a, b, c]: [Xyz; 3]) -> Option<(f64, f64)> {
    let delta = |p: Xyz| [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let [ux, uy, uz] = delta(b);
    let [vx, vy, vz] = delta(c);
    let [x, y, z] = [uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx];
    if 0.5 * x + 0.87 * y + 0.5652 * z <= 0. {
        return None;
    }
    let light = (0.48 * x + 0.72 * y + 0.5 * z) / x.hypot(y).hypot(z);
    let depth = (project(a)[2] + project(b)[2] + project(c)[2]) / 3.;
    Some((depth, light))
}

// The chamfer's upper edge is offset inwards by its full 45-degree bevel width.
fn inset_plate(width: f64) -> [Xy; PLATE.len()] {
    let normal = |[x0, y0]: Xy, [x1, y1]: Xy| {
        let length = (x1 - x0).hypot(y1 - y0);
        [(y0 - y1) / length, (x1 - x0) / length]
    };
    std::array::from_fn(|i| {
        let [x, y] = PLATE[i];
        let a = normal(PLATE[(i + PLATE.len() - 1) % PLATE.len()], [x, y]);
        let b = normal([x, y], PLATE[(i + 1) % PLATE.len()]);
        let miter = width / (1. + a[0] * b[0] + a[1] * b[1]);
        [x + (a[0] + b[0]) * miter, y + (a[1] + b[1]) * miter]
    })
}

// Engraving and carving need an actual recess in the model so the toolpath is
// visible only through the machined opening. Simulate the V-bit, then mesh a
// small patch of the face around the cut; the rest stays a solid plate.
fn v_groove_model(program: &Program, tool: &Tool) -> Model {
    let mut with_tool = Program::new();
    with_tool.push(Step::ToolChange { tool: tool.number });
    with_tool.extend(program.clone());
    let ToolKind::VBit {
        included_angle_deg,
        tip_radius,
    } = tool.kind
    else {
        unreachable!("V-groove examples use a V-bit");
    };
    let radius = tool.radius();
    let pitch = 0.25;
    let field = cam_sim::simulate(
        &with_tool,
        [-20., -13., -6.],
        [20., 13., 0.],
        &SimOptions {
            resolution: pitch,
            tool_radius: radius,
        },
        &[SimTool {
            number: tool.number,
            profile: ToolProfile {
                radius,
                shape: ProfileShape::VTip {
                    half_angle_rad: included_angle_deg.to_radians() / 2.,
                    tip_radius,
                },
            },
            noncutting_tip_radius: None,
        }],
    )
    .field;
    let mesh = field.to_mesh();
    let mut model = Model::default();

    // A shallow sampling allowance applies only to the simulated groove
    // surface. The tool tip can fall between cells at the bottom of a V.
    let sampled_z = |p: [f32; 3]| {
        let mut p = p.map(f64::from);
        if p[2] < 0. {
            p[2] -= pitch * 0.5;
        }
        p
    };
    let (nx, ny) = field.dims();
    let mut patches = Vec::new();
    let mut opening = Vec::new();
    for y in 0..ny - 1 {
        let row = y * nx;
        let is_cut = |x: usize| {
            [row + x, row + x + 1, row + nx + x + 1, row + nx + x]
                .iter()
                .any(|&i| mesh.positions[i][2] < 0.)
        };
        let mut x = 0;
        while x < nx - 1 {
            if !is_cut(x) {
                x += 1;
                continue;
            }
            let start = x;
            while x < nx - 1 && is_cut(x) {
                let a = row + x;
                for indices in [[a, a + 1, a + nx + 1], [a, a + nx + 1, a + nx]] {
                    let triangle = indices.map(|i| sampled_z(mesh.positions[i]));
                    let light = indices
                        .iter()
                        .map(|&i| {
                            let [x, y, z] = mesh.normals[i];
                            -x * 0.25 - y * 0.3 + z * 0.45
                        })
                        .sum::<f32>()
                        / 3.;
                    let gray = (158. + 95. * light).clamp(115., 219.) as u8;
                    patches.push((
                        triangle.to_vec(),
                        format!("#{gray:02x}{:02x}{:02x}", gray + 7, gray + 12),
                    ));
                }
                x += 1;
            }
            let (a, b) = (row + start, row + x);
            let points = [a, b, b + nx, a + nx].map(|i| {
                let [x, y, _] = mesh.positions[i].map(f64::from);
                [x, y]
            });
            opening.push(Polygon::new(contour(&points)).unwrap());
        }
    }
    let opening = cam_geo::union(&opening, &[]).unwrap();
    let ring = |contour: &Contour| contour.points().iter().map(|p| [p.x, p.y, 0.]).collect();
    for region in &opening {
        // Solid fill beneath the tessellation keeps the grooves opaque even
        // along antialiased edges shared by neighboring triangles.
        model.faces.push(Face {
            outer: ring(region.outer()),
            holes: region.holes().iter().map(ring).collect(),
            fill: "#a5b2bf".into(),
            edge: false,
            occludes: false,
        });
    }
    let sort_depth =
        |p: &[Xyz]| p.iter().map(|&[x, y, _]| 0.5 * x + 0.87 * y).sum::<f64>() / p.len() as f64;
    patches.sort_by(|(a, _), (b, _)| sort_depth(a).total_cmp(&sort_depth(b)));
    for (vertices, fill) in patches {
        model.face(vertices, &fill, false);
    }
    model.walls(&FRAME, 0., -6., false);
    for region in cam_geo::difference(&[Polygon::new(contour(&FRAME)).unwrap()], &opening).unwrap()
    {
        model.faces.push(Face {
            outer: ring(region.outer()),
            holes: region.holes().iter().map(ring).collect(),
            fill: "#e1e7eb".into(),
            edge: true,
            occludes: true,
        });
    }
    model
}

const DOME_RADIUS: f64 = 18.;
const DOME_HEIGHT: f64 = 8.;

fn dome(model: &mut Model) -> ParallelGeometry {
    let rim = circle([0., 0.], DOME_RADIUS);
    let ring = |n: usize, i: usize| {
        let t = n as f64 / 10.;
        let [x, y] = rim[i % rim.len()];
        [x * t, y * t, DOME_HEIGHT * (1. - t * t)]
    };
    let mut triangles = vec![];
    for n in 0..10 {
        for i in 0..rim.len() {
            let a = ring(n, i);
            let b = ring(n + 1, i);
            let c = ring(n + 1, i + 1);
            triangles.push([a, b, c]);
            if n > 0 {
                triangles.push([a, c, ring(n, i + 1)]);
            }
        }
    }
    model.walls(&rim, 0., -4., false);
    // Nested shaded silhouettes render the same rings without visible mesh edges.
    for n in (1..=10).rev() {
        let shade = 202 + ((10 - n) * 3) as u8;
        model.face(
            (0..rim.len()).map(|i| ring(n, i)).collect(),
            &format!("#{shade:02x}{:02x}{:02x}", shade + 7, shade + 12),
            false,
        );
        model.faces.last_mut().unwrap().occludes = false;
    }
    model.curved_surface.clone_from(&triangles);
    model.guides.push(at(&rim, 0.));
    ParallelGeometry {
        selected: triangles.clone(),
        triangles,
        error_bound: 0.,
        heights: CuttingHeights {
            top: DOME_HEIGHT,
            bottom: 0.,
        },
    }
}
