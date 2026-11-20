//! Hidden line removal for orthographic backplots.
use std::ops::Range;

use super::{project, Xy, Xyz};

const EPS: f64 = 1e-8;

pub(super) struct Occluder {
    outline: Vec<Xy>,
    holes: Vec<Vec<Xy>>,
    bounds: [Xy; 2],
    origin: Xyz,
    slope: Xy,
}

impl Occluder {
    pub(super) fn new(outline: &[Xyz], holes: &[Vec<Xyz>]) -> Option<Self> {
        let projected: Vec<_> = outline.iter().copied().map(project).collect();
        let origin = projected[0];
        let slope = projected[1..].windows(2).find_map(|pair| {
            let [ux, uy, uz] = sub(pair[0], origin);
            let [vx, vy, vz] = sub(pair[1], origin);
            let det = ux * vy - vx * uy;
            (det.abs() > EPS).then(|| [(uz * vy - vz * uy) / det, (ux * vz - vx * uz) / det])
        })?;
        let outline: Vec<_> = projected.into_iter().map(xy).collect();
        let mut bounds = [outline[0], outline[0]];
        for &[x, y] in &outline[1..] {
            bounds[0] = [bounds[0][0].min(x), bounds[0][1].min(y)];
            bounds[1] = [bounds[1][0].max(x), bounds[1][1].max(y)];
        }
        Some(Self {
            bounds,
            outline,
            holes: holes
                .iter()
                .map(|hole| hole.iter().map(|&p| xy(project(p))).collect())
                .collect(),
            origin,
            slope,
        })
    }

    fn depth_delta(&self, [x, y, depth]: Xyz) -> f64 {
        self.origin[2] + (x - self.origin[0]) * self.slope[0] + (y - self.origin[1]) * self.slope[1]
            - depth
    }

    fn hidden_ranges(&self, a: Xyz, b: Xyz, breaks: &mut Vec<f64>, out: &mut Vec<Range<f64>>) {
        if (0..2).any(|i| a[i].max(b[i]) < self.bounds[0][i] || a[i].min(b[i]) > self.bounds[1][i])
        {
            return;
        }
        // A path on the surface is visible. Only material strictly closer to
        // the viewer can hide a stroke.
        let near_a = self.depth_delta(a) - EPS;
        let near_b = self.depth_delta(b) - EPS;
        if near_a <= 0. && near_b <= 0. {
            return;
        }
        let [dx, dy, _] = sub(b, a);
        let dir = [dx, dy];
        breaks.clear();
        breaks.extend([0., 1.]);
        if near_a.signum() != near_b.signum() {
            breaks.push(near_a / (near_a - near_b));
        }
        for ring in std::iter::once(&self.outline).chain(&self.holes) {
            for (&p, &q) in ring.iter().zip(ring.iter().cycle().skip(1)) {
                let edge = [q[0] - p[0], q[1] - p[1]];
                let offset = [p[0] - a[0], p[1] - a[1]];
                let det = cross(dir, edge);
                if det.abs() > EPS {
                    let along_edge = cross(offset, dir) / det;
                    let along_path = cross(offset, edge) / det;
                    if (0. ..=1.).contains(&along_edge) && (0. ..1.).contains(&along_path) {
                        breaks.push(along_path);
                    }
                }
            }
        }
        // Each interval lies entirely on one side of every edge and of the face's plane.
        breaks.sort_unstable_by(f64::total_cmp);
        for pair in breaks.windows(2) {
            let (start, end) = (pair[0], pair[1]);
            if end - start <= EPS {
                continue;
            }
            let t = (start + end) / 2.;
            if near_a + t * (near_b - near_a) <= 0. {
                continue;
            }
            let at = [a[0] + t * dx, a[1] + t * dy];
            if inside(at, &self.outline) && !self.holes.iter().any(|ring| inside(at, ring)) {
                out.push(start..end);
            }
        }
    }
}

#[derive(Default)]
pub(super) struct Clipper {
    hidden: Vec<Range<f64>>,
    breaks: Vec<f64>,
}

impl Clipper {
    pub(super) fn visible(
        &mut self,
        a: Xyz,
        b: Xyz,
        occluders: &[Occluder],
        mut emit: impl FnMut(Xyz, Xyz),
    ) {
        self.hidden.clear();
        let (pa, pb) = (project(a), project(b));
        for face in occluders {
            face.hidden_ranges(pa, pb, &mut self.breaks, &mut self.hidden);
        }
        self.hidden
            .sort_unstable_by(|a, b| a.start.total_cmp(&b.start));
        let at = |t: f64| {
            if t == 0. {
                a
            } else if t == 1. {
                b
            } else {
                std::array::from_fn(|i| a[i] + t * (b[i] - a[i]))
            }
        };
        let mut cursor: f64 = 0.;
        for range in &self.hidden {
            if range.start - cursor > EPS {
                emit(at(cursor), at(range.start));
            }
            cursor = cursor.max(range.end);
        }
        if 1. - cursor > EPS {
            emit(at(cursor), b);
        }
    }
}

fn inside([x, y]: Xy, ring: &[Xy]) -> bool {
    let mut hit = false;
    for (&[ax, ay], &[bx, by]) in ring.iter().zip(ring.iter().cycle().skip(1)) {
        if (ay > y) != (by > y) && x < ax + (y - ay) * (bx - ax) / (by - ay) {
            hit = !hit;
        }
    }
    hit
}

fn xy([x, y, _]: Xyz) -> Xy {
    [x, y]
}

fn sub(a: Xyz, b: Xyz) -> Xyz {
    std::array::from_fn(|i| a[i] - b[i])
}

fn cross([ax, ay]: Xy, [bx, by]: Xy) -> f64 {
    ax * by - ay * bx
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stock_clips_paths_at_its_edge_but_leaves_the_opening_and_surface_visible() {
        let rectangle =
            |min: f64, max: f64| [[min, -8., 0.], [max, -8., 0.], [max, 8., 0.], [min, 8., 0.]];
        let stock = Occluder::new(&rectangle(-5., 5.), &[rectangle(-1., 1.).to_vec()]).unwrap();
        let mut clipper = Clipper::default();
        let mut visible = vec![];
        clipper.visible(
            [-8., 0., -1.],
            [8., 0., -1.],
            std::slice::from_ref(&stock),
            |a, b| {
                visible.push((a, b));
            },
        );
        assert_eq!(visible.len(), 3);
        assert!(visible[0].1[0] < -5.);
        assert!((visible[1].1[0] - visible[1].0[0] - 2.).abs() < EPS);
        assert!(visible[2].0[0] < 5.);

        visible.clear();
        clipper.visible([-8., 0., 0.], [8., 0., 0.], &[stock], |a, b| {
            visible.push((a, b))
        });
        assert_eq!(visible, vec![([-8., 0., 0.], [8., 0., 0.])]);
    }
}
