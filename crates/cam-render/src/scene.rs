//! The viewport scene: colored polylines for the part outline and the toolpath
//! backplot, plus the pure mapping to GPU vertices. Nothing here touches a GPU,
//! so it is unit-tested like the rest of the pipeline.

use cam_cldata::{MoveKind, Point3, Program, Step};
use cam_geo::{Arc, Point, Polygon};

/// RGBA colour, linear, components in `0.0..=1.0`.
pub type Color = [f32; 4];

/// Part / stock outline colour.
pub const PART: Color = [0.80, 0.82, 0.88, 1.0];
/// Rapid / linking moves (non-cutting).
pub const RAPID: Color = [0.85, 0.75, 0.20, 1.0];
/// Cutting moves.
pub const CUT: Color = [0.25, 0.75, 0.35, 1.0];
/// Plunge / lead-in moves.
pub const PLUNGE: Color = [0.90, 0.35, 0.25, 1.0];

/// Chord tolerance (mm) used when flattening backplot arcs for display.
const ARC_TOL: f64 = 0.02;

/// The colour a backplot move is drawn in, by its role.
fn color_for(kind: MoveKind) -> Color {
    match kind {
        MoveKind::Link | MoveKind::Retract => RAPID,
        MoveKind::Cutting => CUT,
        MoveKind::Plunge | MoveKind::LeadIn => PLUNGE,
    }
}

/// A vertex handed to the GPU: position (mm) and colour. Plain data; the `gpu`
/// feature makes it `bytemuck`-castable for upload.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "gpu", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

/// A connected run of points drawn in one colour.
#[derive(Clone, Debug, PartialEq)]
pub struct LineStrip {
    pub points: Vec<[f32; 3]>,
    pub color: Color,
}

/// A drawable scene: a set of colored polylines.
#[derive(Clone, Debug, Default)]
pub struct Scene {
    pub strips: Vec<LineStrip>,
}

impl Scene {
    /// An empty scene.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a polyline strip.
    pub fn add_strip(&mut self, points: Vec<[f32; 3]>, color: Color) {
        if points.len() >= 2 {
            self.strips.push(LineStrip { points, color });
        }
    }

    /// Add a filled region's outlines (outer boundary and every hole), closed.
    pub fn add_region(&mut self, region: &Polygon, color: Color) {
        self.add_ring(region.outer().points(), color);
        for hole in region.holes() {
            self.add_ring(hole.points(), color);
        }
    }

    fn add_ring(&mut self, pts: &[Point], color: Color) {
        if pts.len() < 3 {
            return;
        }
        let mut strip: Vec<[f32; 3]> = pts.iter().map(|p| [p.x as f32, p.y as f32, 0.0]).collect();
        strip.push(strip[0]); // close the loop
        self.add_strip(strip, color);
    }

    /// Build a backplot from a CL-data program: one colored segment per motion,
    /// arcs flattened, colored by [`MoveKind`].
    pub fn from_program(program: &Program) -> Scene {
        let mut scene = Scene::new();
        let mut cur: Option<Point3> = None;
        for step in program.steps() {
            match step {
                Step::Rapid { to, tag } => {
                    push_segment(&mut scene, cur, *to, color_for(tag.kind));
                    cur = Some(*to);
                }
                Step::Linear { to, tag, .. } => {
                    push_segment(&mut scene, cur, *to, color_for(tag.kind));
                    cur = Some(*to);
                }
                Step::Arc {
                    end,
                    center,
                    dir,
                    tag,
                    ..
                } => {
                    if let Some(start) = cur {
                        push_arc(&mut scene, start, *end, *center, *dir, color_for(tag.kind));
                    }
                    cur = Some(*end);
                }
                Step::Drill(cycle) => {
                    // A drilled hole shows as a plunge marker at each point.
                    for &[x, y] in &cycle.points {
                        scene.add_strip(
                            vec![
                                [x as f32, y as f32, cycle.z_top as f32],
                                [x as f32, y as f32, cycle.depth as f32],
                            ],
                            color_for(cycle.tag.kind),
                        );
                    }
                    cur = None;
                }
                _ => {}
            }
        }
        scene
    }

    /// Expand the strips into a flat `LineList` vertex buffer (two vertices per
    /// segment).
    pub fn line_vertices(&self) -> Vec<Vertex> {
        let mut out = Vec::new();
        for strip in &self.strips {
            for pair in strip.points.windows(2) {
                out.push(Vertex {
                    position: pair[0],
                    color: strip.color,
                });
                out.push(Vertex {
                    position: pair[1],
                    color: strip.color,
                });
            }
        }
        out
    }

    /// Axis-aligned bounds of everything in the scene, or `None` if empty.
    pub fn bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut min = [f32::MAX; 3];
        let mut max = [f32::MIN; 3];
        let mut any = false;
        for strip in &self.strips {
            for p in &strip.points {
                any = true;
                for i in 0..3 {
                    min[i] = min[i].min(p[i]);
                    max[i] = max[i].max(p[i]);
                }
            }
        }
        any.then_some((min, max))
    }
}

fn push_segment(scene: &mut Scene, from: Option<Point3>, to: Point3, color: Color) {
    if let Some(a) = from {
        scene.add_strip(
            vec![
                [a.x as f32, a.y as f32, a.z as f32],
                [to.x as f32, to.y as f32, to.z as f32],
            ],
            color,
        );
    }
}

fn push_arc(
    scene: &mut Scene,
    start: Point3,
    end: Point3,
    center: Point3,
    dir: cam_cldata::ArcDir,
    color: Color,
) {
    let r = ((start.x - center.x).powi(2) + (start.y - center.y).powi(2)).sqrt();
    let a0 = (start.y - center.y).atan2(start.x - center.x);
    let a1 = (end.y - center.y).atan2(end.x - center.x);
    let ccw = matches!(dir, cam_cldata::ArcDir::Ccw);
    let xy = Arc::new(Point::new(center.x, center.y), r, a0, a1, ccw).flatten(ARC_TOL);
    let n = xy.len().max(2);
    let pts: Vec<[f32; 3]> = xy
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let t = i as f64 / (n - 1) as f64;
            let z = start.z + (end.z - start.z) * t;
            [p.x as f32, p.y as f32, z as f32]
        })
        .collect();
    scene.add_strip(pts, color);
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_cldata::ProgramBuilder;

    #[test]
    fn backplot_colors_moves_by_kind() {
        let prog = ProgramBuilder::new()
            .op(0)
            .feed(300.0)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .linear(Point3::new(0.0, 0.0, -1.0), MoveKind::Plunge)
            .linear(Point3::new(10.0, 0.0, -1.0), MoveKind::Cutting)
            .build();
        let scene = Scene::from_program(&prog);
        // Rapid has no prior position ⇒ no segment; then plunge + cut = 2 strips.
        assert_eq!(scene.strips.len(), 2);
        assert_eq!(scene.strips[0].color, PLUNGE);
        assert_eq!(scene.strips[1].color, CUT);
    }

    #[test]
    fn rapid_between_points_is_drawn_in_rapid_color() {
        let prog = ProgramBuilder::new()
            .op(0)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
            .build();
        let scene = Scene::from_program(&prog);
        assert_eq!(scene.strips.len(), 1);
        assert_eq!(scene.strips[0].color, RAPID);
    }

    #[test]
    fn line_vertices_are_two_per_segment() {
        let mut scene = Scene::new();
        scene.add_strip(vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]], CUT);
        // A 3-point strip is 2 segments ⇒ 4 vertices.
        assert_eq!(scene.line_vertices().len(), 4);
    }

    #[test]
    fn bounds_span_all_points() {
        let mut scene = Scene::new();
        scene.add_strip(vec![[0.0, 0.0, -2.0], [10.0, 5.0, 3.0]], CUT);
        let (min, max) = scene.bounds().unwrap();
        assert_eq!(min, [0.0, 0.0, -2.0]);
        assert_eq!(max, [10.0, 5.0, 3.0]);
    }
}
