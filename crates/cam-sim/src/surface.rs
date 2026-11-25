//! Compact stock meshing. Flat interiors become rectangles; only the changing
//! surface is tessellated. Dual contour vertices recover walls from the actual
//! cutter sweeps instead of following the edges of square heightfield cells.
use super::{Heightfield, SurfaceMesh, MESH_TILE_SIZE};
use rayon::prelude::*;

#[derive(Clone, Copy)]
struct Corner {
    xy: [f32; 2],
    wall_normal: Option<[f32; 2]>,
    flat_cut_rim: bool,
}

impl Corner {
    fn separates(&self, lower: f32, upper: f32, stock_top: f32, wall_step: f32) -> bool {
        upper - lower > wall_step || (self.flat_cut_rim && lower < stock_top && upper == stock_top)
    }
}

// Sweeps near each mesh tile, used to reconstruct the union of overlapping
// cutter passes. Very long sweeps are kept separately to bound index memory.
struct SweepIndex {
    width: usize,
    bins: Vec<Vec<u32>>,
    long: Vec<u32>,
}

impl SweepIndex {
    fn new(field: &Heightfield, cancelled: &dyn Fn() -> bool) -> Option<Self> {
        let width = field.nx.div_ceil(MESH_TILE_SIZE);
        let height = field.ny.div_ceil(MESH_TILE_SIZE);
        let mut index = Self {
            width,
            bins: vec![Vec::new(); width * height],
            long: Vec::new(),
        };
        for (id, cut) in field.cuts.iter().enumerate() {
            if id % 1024 == 0 && cancelled() {
                return None;
            }
            // Thread teeth remove bounded intervals; their sweeps are not
            // ordinary downward cuts and cannot join the heightfield union.
            if matches!(cut.tool.shape, crate::ProfileShape::Thread { .. }) {
                continue;
            }
            let r = cut.tool.radius + field.res * 2.;
            let bin = |coordinate: f64, axis: usize, count: usize| {
                (((coordinate - field.origin[axis]) / (field.res * MESH_TILE_SIZE as f64)).floor()
                    as isize)
                    .clamp(0, count as isize - 1) as usize
            };
            let x0 = bin(cut.a[0].min(cut.b[0]) - r, 0, width);
            let x1 = bin(cut.a[0].max(cut.b[0]) + r, 0, width);
            let y0 = bin(cut.a[1].min(cut.b[1]) - r, 1, height);
            let y1 = bin(cut.a[1].max(cut.b[1]) + r, 1, height);
            if (x1 - x0 + 1) * (y1 - y0 + 1) > 256 {
                index.long.push(id as u32);
            } else {
                for y in y0..=y1 {
                    for x in x0..=x1 {
                        index.bins[y * width + x].push(id as u32);
                    }
                }
            }
        }
        Some(index)
    }

    fn near(&self, x: usize, y: usize) -> impl Iterator<Item = u32> + '_ {
        let bx = (x / MESH_TILE_SIZE).min(self.width - 1);
        let by = (y / MESH_TILE_SIZE).min(self.bins.len() / self.width - 1);
        self.bins[by * self.width + bx]
            .iter()
            .chain(&self.long)
            .copied()
    }
}

/// One mesh per stock tile, rebuilt only when cuts change it or its neighbours.
/// Revisions belong to heightfield snapshots, so rewinds and disposable partial
/// motions cannot reuse surfaces from a different state.
#[derive(Default)]
pub struct StockMeshCache {
    grid: Option<(usize, usize, u64)>,
    tiles: Vec<Option<(u64, SurfaceMesh)>>,
}

impl StockMeshCache {
    /// Rebuild changed tiles concurrently. The cancellation callback may be
    /// polled from multiple worker threads; a cancelled build publishes no mesh.
    pub fn mesh_controlled(
        &mut self,
        field: &Heightfield,
        floor: f64,
        cancelled: &(dyn Fn() -> bool + Sync),
    ) -> Option<SurfaceMesh> {
        if cancelled() {
            return None;
        }
        let key = (field.nx, field.ny, floor.to_bits());
        if self.grid != Some(key) {
            self.grid = Some(key);
            self.tiles = vec![None; field.mesh_revisions.len()];
        }
        let width = field.nx.div_ceil(MESH_TILE_SIZE);
        let sweeps = SweepIndex::new(field, cancelled)?;
        // Tiles read one immutable stock snapshot and own disjoint cache slots.
        // Assembly below stays in tile order, preserving vertex/provenance indices.
        self.tiles
            .par_iter_mut()
            .enumerate()
            .try_for_each(|(index, tile)| {
                if cancelled() {
                    return None;
                }
                let revision = field.mesh_revisions[index];
                if tile.as_ref().is_some_and(|(saved, _)| *saved == revision) {
                    return Some(());
                }
                let x = index % width * MESH_TILE_SIZE;
                let y = index / width * MESH_TILE_SIZE;
                let mesh = field.solid_mesh_region_controlled(
                    floor,
                    [x, y],
                    [
                        (x + MESH_TILE_SIZE).min(field.nx),
                        (y + MESH_TILE_SIZE).min(field.ny),
                    ],
                    cancelled,
                    &sweeps,
                )?;
                *tile = Some((revision, mesh));
                Some(())
            })?;
        let vertices = self
            .tiles
            .iter()
            .flatten()
            .map(|(_, m)| m.positions.len())
            .sum();
        let indices = self
            .tiles
            .iter()
            .flatten()
            .map(|(_, m)| m.indices.len())
            .sum();
        let mut mesh = SurfaceMesh {
            positions: Vec::with_capacity(vertices),
            normals: Vec::with_capacity(vertices),
            machined: Vec::with_capacity(vertices),
            operations: Vec::with_capacity(vertices),
            cut_sources: field.surface_tracking.then(|| Vec::with_capacity(vertices)),
            indices: Vec::with_capacity(indices),
        };
        for (_, tile) in self.tiles.iter().flatten() {
            if cancelled() {
                return None;
            }
            let offset = mesh.positions.len() as u32;
            mesh.positions.extend_from_slice(&tile.positions);
            mesh.normals.extend_from_slice(&tile.normals);
            mesh.machined.extend_from_slice(&tile.machined);
            mesh.operations.extend_from_slice(&tile.operations);
            if let (Some(sources), Some(tile_sources)) = (&mut mesh.cut_sources, &tile.cut_sources)
            {
                sources.extend_from_slice(tile_sources);
            }
            mesh.indices.extend(tile.indices.iter().map(|i| i + offset));
        }
        (!cancelled()).then_some(mesh)
    }
}
impl SurfaceMesh {
    fn top_quad(
        &mut self,
        p: [[f32; 3]; 4],
        n: [[f32; 3]; 4],
        cut: bool,
        top: f32,
        op: Option<u32>,
    ) {
        let flags = [[0, 1, 2], [0, 2, 3]].map(|ids| cut && ids.iter().any(|&i| p[i][2] < top));
        if flags[0] == flags[1] {
            self.quad(p, n, flags[0], op);
        } else {
            for (ids, cut) in [[0, 1, 2], [0, 2, 3]].into_iter().zip(flags) {
                let base = self.positions.len() as u32;
                self.positions.extend(ids.map(|i| p[i]));
                self.normals.extend(ids.map(|i| n[i]));
                self.machined.extend([cut; 3]);
                self.operations.extend([op.filter(|_| cut); 3]);
                if let Some(sources) = &mut self.cut_sources {
                    sources.extend([None; 3]);
                }
                self.indices.extend([base, base + 1, base + 2]);
            }
        }
    }
    pub(super) fn quad(&mut self, p: [[f32; 3]; 4], n: [[f32; 3]; 4], cut: bool, op: Option<u32>) {
        let base = self.positions.len() as u32;
        self.positions.extend(p);
        self.normals.extend(n);
        self.machined.extend([cut; 4]);
        self.operations.extend([op.filter(|_| cut); 4]);
        if let Some(sources) = &mut self.cut_sources {
            sources.extend([None; 4]);
        }
        self.indices
            .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
}
impl SurfaceMesh {
    pub(super) fn stamp_source(&mut self, first: usize, field: &Heightfield, cut: u32) {
        let Some(sources) = &mut self.cut_sources else {
            return;
        };
        let Some(sweep) = field.cuts.get(cut as usize) else {
            return;
        };
        for (index, point) in self.positions.iter().enumerate().skip(first) {
            if self.machined[index] {
                sources[index] = sweep.surface_location(*point);
            }
        }
    }
}
impl Heightfield {
    fn top_cut(&self, x: usize, y: usize, wall_step: f32) -> u32 {
        let cut = self.winners[y * self.nx + x];
        if cut != u32::MAX || self.operation_at(x, y).is_some() {
            return cut;
        }
        let z = self.z[y * self.nx + x];
        (-1..=1)
            .flat_map(|dy| (-1..=1).map(move |dx| (dx, dy)))
            .filter_map(|(dx, dy)| {
                let (nx, ny) = (x as isize + dx, y as isize + dy);
                let height = self.height(nx, ny)?;
                let cut = self.winners[ny as usize * self.nx + nx as usize];
                (height < self.top && (z - height).abs() <= wall_step && cut != u32::MAX)
                    .then_some((height, cut))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map_or(u32::MAX, |(_, cut)| cut)
    }
    /// Smoothing can lower an uncut cell's displayed edge beside a shallow cut.
    /// Give that fringe the neighbouring cut's owner, just as its height/normal
    /// come from the compatible neighbouring surface.
    fn top_operation(&self, x: usize, y: usize, wall_step: f32) -> Option<u32> {
        self.operation_at(x, y).or_else(|| {
            let z = self.z[y * self.nx + x];
            (-1..=1)
                .flat_map(|dy| (-1..=1).map(move |dx| (dx, dy)))
                .filter_map(|(dx, dy)| {
                    let (nx, ny) = (x as isize + dx, y as isize + dy);
                    let height = self.height(nx, ny)?;
                    if height < self.top && (z - height).abs() <= wall_step {
                        self.operation_at(nx as usize, ny as usize)
                            .map(|op| (height, op))
                    } else {
                        None
                    }
                })
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, op)| op)
        })
    }

    fn height(&self, x: isize, y: isize) -> Option<f32> {
        if x < 0 || y < 0 || x >= self.nx as isize || y >= self.ny as isize {
            None
        } else {
            Some(self.z[y as usize * self.nx + x as usize])
        }
    }
    pub(super) fn grid_xy(&self, x: usize, y: usize) -> [f32; 2] {
        [
            (self.origin[0] + x as f64 * self.res).min(self.max[0]) as f32,
            (self.origin[1] + y as f64 * self.res).min(self.max[1]) as f32,
        ]
    }
    fn contour_corner(&self, x: usize, y: usize, wall_step: f32, sweeps: &SweepIndex) -> Corner {
        let mut corner = Corner {
            xy: self.grid_xy(x, y),
            wall_normal: None,
            flat_cut_rim: false,
        };
        let left = x.saturating_sub(1);
        let right = x.min(self.nx - 1);
        let below = y.saturating_sub(1) * self.nx;
        let above = y.min(self.ny - 1) * self.nx;
        let ids = [below + left, below + right, above + right, above + left];
        let heights = ids.map(|i| self.z[i]);
        if heights.iter().all(|&z| z == heights[0]) {
            return corner;
        }
        corner.flat_cut_rim = self.flat_cut_rim(ids, heights);
        if x == 0 || y == 0 || x == self.nx || y == self.ny {
            return corner;
        }
        let mut sorted = heights;
        sorted.sort_unstable_by(f32::total_cmp);
        let Some([lower, upper]) = (0..3)
            .map(|i| [sorted[i], sorted[i + 1]])
            .filter(|&[low, high]| corner.separates(low, high, self.top, wall_step))
            .max_by(|a, b| (a[1] - a[0]).total_cmp(&(b[1] - b[0])))
        else {
            return corner;
        };
        let level = (f64::from(lower) + f64::from(upper)) * 0.5;
        let low = heights.map(|h| f64::from(h) < level);
        // A saddle contains two separate contours; a single dual vertex must not
        // connect the two voids or erase the intervening material.
        if low[0] == low[2] && low[1] == low[3] {
            return corner;
        }
        let mut cutters = Vec::with_capacity(4);
        for i in 0..4 {
            let id = self.winners[ids[i]];
            if low[i] && id != u32::MAX && !cutters.contains(&id) {
                cutters.push(id);
            }
        }
        if cutters.is_empty() {
            return corner;
        }
        // A corner can border several overlapping segments of a rolled tool
        // path. The winner at a cell center alone does not describe their union.
        let xy = corner.xy.map(f64::from);
        for id in sweeps.near(x, y) {
            let cut = &self.cuts[id as usize];
            let radius = cut.tool.radius + self.res * 2.;
            if xy[0] >= cut.a[0].min(cut.b[0]) - radius
                && xy[0] <= cut.a[0].max(cut.b[0]) + radius
                && xy[1] >= cut.a[1].min(cut.b[1]) - radius
                && xy[1] <= cut.a[1].max(cut.b[1]) + radius
                && !cutters.contains(&id)
            {
                cutters.push(id);
            }
        }
        let implicit = |p: [f64; 2]| {
            cutters
                .iter()
                .map(|&id| self.cuts[id as usize].implicit(p[0], p[1], level))
                .fold(f64::INFINITY, f64::min)
        };
        let centers = [
            self.center(x - 1, y - 1),
            self.center(x, y - 1),
            self.center(x, y),
            self.center(x - 1, y),
        ];
        let origin = corner.xy.map(f64::from);
        // Solve a small Hermite QEF in local coordinates. The weak centre anchor
        // handles parallel normals without drifting along straight walls.
        let mut aa = 1e-5;
        let mut ab = 0.;
        let mut bb = 1e-5;
        let mut rhs = [0.; 2];
        let mut normal = [0.; 2];
        let mut count = 0;
        for i in 0..4 {
            let j = (i + 1) % 4;
            if low[i] == low[j] {
                continue;
            }
            let mut a = [centers[i].0, centers[i].1];
            let mut b = [centers[j].0, centers[j].1];
            let inside = implicit(a) <= 0.;
            if inside == (implicit(b) <= 0.) {
                continue;
            }
            for _ in 0..12 {
                let m = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
                if (implicit(m) <= 0.) == inside {
                    a = m;
                } else {
                    b = m;
                }
            }
            let p = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
            let e = self.res * 0.001;
            let mut n = [
                implicit([p[0] + e, p[1]]) - implicit([p[0] - e, p[1]]),
                implicit([p[0], p[1] + e]) - implicit([p[0], p[1] - e]),
            ];
            let len = n[0].hypot(n[1]);
            if len < 1e-12 {
                continue;
            }
            n = n.map(|v| v / len);
            let d = n[0] * (p[0] - origin[0]) + n[1] * (p[1] - origin[1]);
            aa += n[0] * n[0];
            ab += n[0] * n[1];
            bb += n[1] * n[1];
            rhs[0] += n[0] * d;
            rhs[1] += n[1] * d;
            normal[0] -= n[0];
            normal[1] -= n[1];
            count += 1;
        }
        if count > 0 {
            let det = aa * bb - ab * ab;
            let delta = [
                (rhs[0] * bb - rhs[1] * ab) / det,
                (rhs[1] * aa - rhs[0] * ab) / det,
            ];
            corner.xy = std::array::from_fn(|i| {
                (origin[i] + delta[i]).clamp(
                    [centers[0].0, centers[0].1][i],
                    [centers[2].0, centers[2].1][i],
                ) as f32
            });
            let len = normal[0].hypot(normal[1]);
            if len > 1e-9 {
                corner.wall_normal = Some([(normal[0] / len) as f32, (normal[1] / len) as f32]);
            }
        }
        corner
    }

    fn flat_cut_rim(&self, ids: [usize; 4], heights: [f32; 4]) -> bool {
        if !self.uncut_z.is_empty() || !heights.contains(&self.top) {
            return false;
        }
        // A flat cutter can leave a vertical wall even when the cut is shallow.
        // Use the footprint to distinguish its rim from a ramp meeting the stock plane.
        (0..4).any(|i| {
            let j = (i + 1) % 4;
            let (low, high) = if heights[i] < heights[j] {
                (i, j)
            } else {
                (j, i)
            };
            heights[low] < self.top
                && heights[high] == self.top
                && self
                    .cuts
                    .get(self.winners[ids[low]] as usize)
                    .is_some_and(|cut| {
                        let (cx, cy) = self.center(ids[high] % self.nx, ids[high] / self.nx);
                        cut.tool.shape == crate::ProfileShape::Flat
                            && cut.projection(cx, cy).1 > cut.tool.radius.powi(2)
                    })
        })
    }

    /// Surface at a shared corner, excluding neighbours across a vertical wall.
    /// Compatible slopes share heights and normals, so curved floors shade smoothly.
    fn corner_surface(
        &self,
        x: usize,
        y: usize,
        z: f32,
        wall_step: f32,
        corner: Corner,
    ) -> (f32, [f32; 3], bool) {
        let (x, y) = (x as isize, y as isize);
        let mut h =
            [(x - 1, y - 1), (x, y - 1), (x, y), (x - 1, y)].map(|(i, j)| self.height(i, j));
        let mut sorted = h.map(|v| v.unwrap_or(z));
        sorted.sort_unstable_by(f32::total_cmp);
        let mut low = f32::NEG_INFINITY;
        let mut high = f32::INFINITY;
        for pair in sorted.windows(2) {
            if corner.separates(pair[0], pair[1], self.top, wall_step) {
                if z <= pair[0] {
                    high = high.min(pair[0]);
                }
                if z >= pair[1] {
                    low = low.max(pair[1]);
                }
            }
        }
        h = h.map(|v| v.filter(|v| *v >= low && *v <= high));
        let mut sum = 0.;
        let mut count = 0.;
        let mut cut = false;
        for (i, v) in h.into_iter().enumerate() {
            if let Some(v) = v {
                sum += v;
                count += 1.;
                let (cx, cy) = [(x - 1, y - 1), (x, y - 1), (x, y), (x - 1, y)][i];
                cut |= self.top_is_cut(cy as usize * self.nx + cx as usize);
            }
        }
        let derivative = |edges: [(usize, usize); 2]| {
            let mut sum = 0.;
            let mut count = 0.;
            for (a, b) in edges {
                if let (Some(a), Some(b)) = (h[a], h[b]) {
                    sum += (b - a) / self.res as f32;
                    count += 1.;
                }
            }
            if count > 0. {
                sum / count
            } else {
                0.
            }
        };
        let dx = derivative([(0, 1), (3, 2)]);
        let dy = derivative([(0, 3), (1, 2)]);
        let len = (dx * dx + dy * dy + 1.).sqrt();
        (sum / count, [-dx / len, -dy / len, 1. / len], cut)
    }
    /// Triangulate a solid stock with analytic sub-cell wall contours, shared
    /// smooth slope normals, and merged flat interiors. Verification retains the
    /// original sampled heights; contour recovery changes only the display mesh.
    pub fn to_solid_mesh(&self, floor: f64) -> SurfaceMesh {
        self.to_solid_mesh_controlled(floor, &|| false).unwrap()
    }
    pub fn to_solid_mesh_controlled(
        &self,
        floor: f64,
        cancelled: &dyn Fn() -> bool,
    ) -> Option<SurfaceMesh> {
        let sweeps = SweepIndex::new(self, cancelled)?;
        self.solid_mesh_region_controlled(floor, [0, 0], [self.nx, self.ny], cancelled, &sweeps)
    }

    fn solid_mesh_region_controlled(
        &self,
        floor: f64,
        [x0, y0]: [usize; 2],
        [x1, y1]: [usize; 2],
        cancelled: &dyn Fn() -> bool,
        sweeps: &SweepIndex,
    ) -> Option<SurfaceMesh> {
        let width = x1 - x0;
        let height = y1 - y0;
        let corner_width = width + 1;
        let f = floor as f32;
        let wall_step = self.res as f32 * 2.5;
        let mut corners = Vec::with_capacity(corner_width * (height + 1));
        for y in y0..=y1 {
            if cancelled() {
                return None;
            }
            for x in x0..=x1 {
                corners.push(self.contour_corner(x, y, wall_step, sweeps));
            }
        }
        let mut flat = vec![0u8; width * height];
        for y in y0..y1 {
            if cancelled() {
                return None;
            }
            for x in x0..x1 {
                let z = self.z[y * self.nx + x];
                flat[(y - y0) * width + x - x0] = u8::from(
                    !self.detailed_column(x, y)
                        && !(self.surface_tracking && z < self.top)
                        && z > f + 1e-4
                        && (-1..=1).all(|dy| {
                            (-1..=1).all(|dx| {
                                self.height(x as isize + dx, y as isize + dy)
                                    .is_none_or(|v| v == z)
                            })
                        }),
                );
            }
        }
        let mut mesh = SurfaceMesh {
            cut_sources: self.surface_tracking.then(Vec::new),
            ..Default::default()
        };
        let up = [[0., 0., 1.]; 4];
        let down = [[0., 0., -1.]; 4];
        // Greedy rectangles leave a one-cell collar around changing heights.
        // That collar shares all contour vertices with walls and neighbouring tops.
        for y in y0..y1 {
            if cancelled() {
                return None;
            }
            for x in x0..x1 {
                let id = (y - y0) * width + x - x0;
                if flat[id] != 1 {
                    continue;
                }
                let z = self.z[y * self.nx + x];
                let op = self.operation_at(x, y);
                let mut end_x = x + 1;
                while end_x < x1
                    && flat[(y - y0) * width + end_x - x0] == 1
                    && self.z[y * self.nx + end_x] == z
                    && self.operation_at(end_x, y) == op
                {
                    end_x += 1;
                }
                let mut end_y = y + 1;
                while end_y < y1
                    && (x..end_x).all(|i| {
                        flat[(end_y - y0) * width + i - x0] == 1
                            && self.z[end_y * self.nx + i] == z
                            && self.operation_at(i, end_y) == op
                    })
                {
                    if cancelled() {
                        return None;
                    }
                    end_y += 1;
                }
                for j in y..end_y {
                    for i in x..end_x {
                        flat[(j - y0) * width + i - x0] = 2;
                    }
                }
                // Mark merged cells separately, without allocating another bitmap.
                let a = self.grid_xy(x, y);
                let b = self.grid_xy(end_x, end_y);
                mesh.quad(
                    [
                        [a[0], a[1], z],
                        [b[0], a[1], z],
                        [b[0], b[1], z],
                        [a[0], b[1], z],
                    ],
                    up,
                    self.top_is_cut(y * self.nx + x),
                    op,
                );
                mesh.quad(
                    [
                        [a[0], a[1], f],
                        [a[0], b[1], f],
                        [b[0], b[1], f],
                        [b[0], a[1], f],
                    ],
                    down,
                    false,
                    None,
                );
            }
        }
        // Boundary cells and slopes retain local detail; merged interiors need no more geometry.
        for y in y0..y1 {
            if cancelled() {
                return None;
            }
            for x in x0..x1 {
                if self.detailed_column(x, y) {
                    let xy = [(x, y), (x + 1, y), (x + 1, y + 1), (x, y + 1)]
                        .map(|(i, j)| corners[(j - y0) * corner_width + i - x0].xy);
                    self.mesh_column(x, y, f, xy, &mut mesh);
                    continue;
                }
                let z = self.z[y * self.nx + x];
                if z <= f + 1e-4 {
                    continue;
                }
                let interior = flat[(y - y0) * width + x - x0] == 2;
                if interior && x > 0 && y > 0 && x + 1 < self.nx && y + 1 < self.ny {
                    continue;
                }
                let coords = [(x, y), (x + 1, y), (x + 1, y + 1), (x, y + 1)];
                let c = coords.map(|(i, j)| corners[(j - y0) * corner_width + i - x0]);
                let h = std::array::from_fn(|i| {
                    let (x, y) = coords[i];
                    self.corner_surface(x, y, z, wall_step, c[i])
                });
                let p: [[f32; 3]; 4] =
                    std::array::from_fn(|i| [c[i].xy[0], c[i].xy[1], h[i].0.max(f)]);
                if !interior {
                    let first = mesh.positions.len();
                    mesh.top_quad(
                        p,
                        h.map(|v| v.1),
                        h.iter().any(|v| v.2),
                        self.top,
                        self.top_operation(x, y, wall_step),
                    );
                    mesh.stamp_source(first, self, self.top_cut(x, y, wall_step));
                    mesh.quad(
                        [p[0], p[3], p[2], p[1]].map(|v| [v[0], v[1], f]),
                        down,
                        false,
                        None,
                    );
                }
                // CCW boundary of this top; reverse it for outward-facing walls.
                for (a, b, dx, dy, axis) in [
                    (0, 3, -1, 0, [-1., 0., 0.]),
                    (2, 1, 1, 0, [1., 0., 0.]),
                    (1, 0, 0, -1, [0., -1., 0.]),
                    (3, 2, 0, 1, [0., 1., 0.]),
                ] {
                    let nb = self.height(x as isize + dx, y as isize + dy);
                    if nb.is_some_and(|v| {
                        v > f + 1e-4
                            && !c[a].separates(v, z, self.top, wall_step)
                            && !c[b].separates(v, z, self.top, wall_step)
                    }) {
                        continue;
                    }
                    let lower = |i: usize| {
                        nb.map_or(f, |v| {
                            let (cx, cy) = coords[i];
                            self.corner_surface(cx, cy, v, wall_step, c[i]).0.max(f)
                        })
                    };
                    let n = |i: usize| {
                        c[i].wall_normal
                            .filter(|n| n[0] * axis[0] + n[1] * axis[1] > 0.01)
                            .map(|[nx, ny]| [nx, ny, 0.])
                            .unwrap_or(axis)
                    };
                    let top = [p[a], p[b]];
                    let mut bottom = [[p[a][0], p[a][1], lower(a)], [p[b][0], p[b][1], lower(b)]];
                    let neighbour_cut = nb.is_some()
                        && self.top_is_cut(
                            (y as isize + dy) as usize * self.nx + (x as isize + dx) as usize,
                        );
                    let mut wall =
                        |upper: [[f32; 3]; 2], lower: [[f32; 3]; 2], op: Option<u32>, cut: u32| {
                            if (0..2).any(|i| upper[i][2] - lower[i][2] > 1e-5) {
                                let first = mesh.positions.len();
                                mesh.quad(
                                    [upper[0], upper[1], lower[1], lower[0]],
                                    [n(a), n(b), n(b), n(a)],
                                    neighbour_cut || op.is_some() || cut != u32::MAX,
                                    op,
                                );
                                mesh.stamp_source(first, self, cut);
                            }
                        };
                    let mut op = None;
                    let mut cut = u32::MAX;
                    if nb.is_some() {
                        let (nx, ny) = ((x as isize + dx) as usize, (y as isize + dy) as usize);
                        op = self.operation_at(nx, ny);
                        cut = self.winners[ny * self.nx + nx];
                        let mut previous = self.previous_layers[ny * self.nx + nx];
                        while let Some(layer) = self.layers.get(previous as usize) {
                            let split = std::array::from_fn(|i| {
                                [
                                    top[i][0],
                                    top[i][1],
                                    layer.height.max(bottom[i][2]).min(top[i][2]),
                                ]
                            });
                            wall(split, bottom, op, cut);
                            bottom = split;
                            op = layer.operation;
                            cut = layer.cut;
                            previous = layer.previous;
                        }
                    }
                    wall(top, bottom, op, cut);
                }
            }
        }
        Some(mesh)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ToolProfile;

    #[test]
    fn rolled_corner_uses_all_nearby_cutter_sweeps() {
        let mut field = Heightfield::new([-344.5, -2.5], [-264.5, 20.], 1.1555555555555554, 0.);
        let corner = [-312.791608967, -0.8];
        let mut cache = StockMeshCache::default();
        for r in [4., 3.] {
            let mut points = vec![[-267.791608967, corner[1] - r], [corner[0], corner[1] - r]];
            for i in 1..=5 {
                let angle =
                    -std::f64::consts::FRAC_PI_2 - i as f64 * std::f64::consts::FRAC_PI_2 / 5.;
                points.push([corner[0] + r * angle.cos(), corner[1] + r * angle.sin()]);
            }
            points.push([corner[0] - r, 10.]);
            for pair in points.windows(2) {
                field.cut_segment(
                    [pair[0][0], pair[0][1], -18.],
                    [pair[1][0], pair[1][1], -18.],
                    3.,
                );
                let cached = cache.mesh_controlled(&field, -18., &|| false).unwrap();
                let fresh = StockMeshCache::default()
                    .mesh_controlled(&field, -18., &|| false)
                    .unwrap();
                assert_eq!(cached, fresh, "incremental mesh differs from fresh mesh");
            }
        }
        let mesh = field.to_solid_mesh(-18.);
        let vertices: Vec<_> = mesh
            .positions
            .iter()
            .zip(&mesh.normals)
            .filter(|(p, n)| {
                n[2].abs() < 0.1
                    && (p[0] as f64 - corner[0]).abs() < 1.
                    && (p[1] as f64 - corner[1]).abs() < 1.
            })
            .map(|(p, _)| *p)
            .collect();
        assert!(!vertices.is_empty());
        for p in vertices {
            assert!(
                p[0] as f64 >= corner[0] - 0.08 && p[1] as f64 >= corner[1] - 0.08,
                "spike at {p:?}"
            );
            assert!(
                (p[0] as f64 - corner[0])
                    .abs()
                    .min((p[1] as f64 - corner[1]).abs())
                    < 0.08,
                "cut-in at {p:?}"
            );
        }
    }

    fn surface_measures(mesh: &SurfaceMesh) -> (f64, f64) {
        let mut area = 0.;
        let mut volume = 0.;
        for triangle in mesh.indices.as_chunks::<3>().0 {
            let [a, b, c] = [0, 1, 2].map(|i| mesh.positions[triangle[i] as usize].map(f64::from));
            let u = std::array::from_fn::<_, 3, _>(|i| b[i] - a[i]);
            let v = std::array::from_fn::<_, 3, _>(|i| c[i] - a[i]);
            let cross = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            area += cross.iter().map(|v| v * v).sum::<f64>().sqrt() * 0.5;
            volume += (0..3).map(|i| a[i] * cross[i]).sum::<f64>() / 6.;
        }
        (area, volume)
    }

    fn check_cached_surface(cache: &mut StockMeshCache, field: &Heightfield, floor: f64) {
        let actual = cache.mesh_controlled(field, floor, &|| false).unwrap();
        let fresh = StockMeshCache::default()
            .mesh_controlled(field, floor, &|| false)
            .unwrap();
        assert_eq!(
            actual, fresh,
            "cached tiles must match a fresh build, including provenance"
        );
        let (area, volume) = surface_measures(&actual);
        let (expected_area, expected_volume) = surface_measures(&field.to_solid_mesh(floor));
        assert!(
            (area - expected_area).abs() < 1e-4,
            "area: {area} vs {expected_area}"
        );
        assert!(
            (volume - expected_volume).abs() < 1e-4,
            "volume: {volume} vs {expected_volume}"
        );
    }

    #[test]
    fn tiled_mesh_is_identical_with_one_or_four_workers() {
        let mut field = Heightfield::new([0., 0.], [48., 48.], 0.5, 0.);
        field.set_surface_tracking(true);
        field.set_cut_source(10, (0., 1.));
        field.cut_segment_profile_controlled(
            [4., 5., -3.],
            [40., 43., -2.],
            &ToolProfile::flat(3.),
            Some(7),
            &|| false,
        );
        let build = |threads| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
                .install(|| {
                    StockMeshCache::default()
                        .mesh_controlled(&field, -6., &|| false)
                        .unwrap()
                })
        };
        assert_eq!(build(1), build(4));
    }

    #[test]
    fn mesh_tiles_reuse_unchanged_regions_and_follow_rewinds_and_replacement_stock() {
        let mut cache = StockMeshCache::default();
        let mut field = Heightfield::new([0., 0.], [48., 48.], 0.5, 0.);
        check_cached_surface(&mut cache, &field, -6.);
        let untouched = cache
            .tiles
            .last()
            .unwrap()
            .as_ref()
            .unwrap()
            .1
            .positions
            .as_ptr();
        field.lower_rect([14., 14.], [18., 18.], -2.);
        check_cached_surface(&mut cache, &field, -6.);
        assert_eq!(
            untouched,
            cache
                .tiles
                .last()
                .unwrap()
                .as_ref()
                .unwrap()
                .1
                .positions
                .as_ptr()
        );
        let earlier = field.clone();
        field.cut_segment_profile_controlled(
            [14., 15.8, -4.],
            [18., 16.2, -4.],
            &ToolProfile {
                radius: 2.,
                shape: crate::ProfileShape::Ball,
            },
            Some(7),
            &|| false,
        );
        check_cached_surface(&mut cache, &field, -6.);
        check_cached_surface(&mut cache, &earlier, -6.);
        field = earlier;
        field.cut_segment_profile_controlled(
            [15.8, 14., -3.],
            [16.2, 18., -3.],
            &ToolProfile::flat(2.),
            Some(8),
            &|| false,
        );
        check_cached_surface(&mut cache, &field, -6.);
        check_cached_surface(&mut cache, &field, -8.);
        let replacement = Heightfield::new([2., 3.], [50., 51.], 0.5, 1.);
        check_cached_surface(&mut cache, &replacement, -6.);
    }

    #[test]
    fn mesh_tiles_preserve_thread_flanks_and_force_sources_across_tile_boundaries() {
        let mut cache = StockMeshCache::default();
        let mut field = Heightfield::new([0., 0.], [48., 48.], 0.5, 0.);
        field.set_surface_tracking(true);
        check_cached_surface(&mut cache, &field, -6.);
        let tool = ToolProfile {
            radius: 3.,
            shape: crate::ProfileShape::Thread {
                root_radius: 1.,
                tooth_height: 1.,
                pitch: 2.,
                tooth_count: 2,
            },
        };
        for (step, (a, b)) in [
            ([14., 15.8, -5.], [18., 16.2, -5.]),
            ([15.8, 14., -4.5], [16.2, 18., -4.5]),
        ]
        .into_iter()
        .enumerate()
        {
            field.set_cut_source(step, (0., 1.));
            field.cut_segment_profile_controlled(a, b, &tool, Some(7), &|| false);
            check_cached_surface(&mut cache, &field, -6.);
        }
        field.set_surface_tracking(false);
        check_cached_surface(&mut cache, &field, -6.);
    }

    #[test]
    fn cancelled_mesh_tile_work_cannot_publish_or_contaminate_an_earlier_frame() {
        let mut cache = StockMeshCache::default();
        let mut field = Heightfield::new([0., 0.], [48., 48.], 0.5, 0.);
        check_cached_surface(&mut cache, &field, -6.);
        let earlier = field.clone();
        field.lower_rect([0., 0.], [48., 48.], -2.);
        let checks = std::sync::atomic::AtomicUsize::new(0);
        assert!(cache
            .mesh_controlled(&field, -6., &|| {
                checks.fetch_add(1, std::sync::atomic::Ordering::Relaxed) > 150
            })
            .is_none());
        check_cached_surface(&mut cache, &earlier, -6.);
        check_cached_surface(&mut cache, &field, -6.);
    }

    #[test]
    fn force_surfaces_retain_the_removing_move_after_an_air_pass() {
        let mut field = Heightfield::new([0., 0.], [20., 10.], 0.5, 0.);
        field.set_surface_tracking(true);
        let tool = ToolProfile::flat(2.);
        field.set_cut_source(3, (0., 1.));
        field.cut_segment_profile_controlled(
            [-4., 5., -2.],
            [24., 5., -2.],
            &tool,
            Some(7),
            &|| false,
        );
        field.set_cut_source(4, (0., 1.));
        field.cut_segment_profile_controlled(
            [24., 5., -2.],
            [-4., 5., -2.],
            &tool,
            Some(7),
            &|| false,
        );
        let mesh = field.to_solid_mesh(-5.);
        let sources = mesh.cut_sources.as_ref().unwrap();
        assert_eq!(sources.len(), mesh.positions.len());
        assert!(sources
            .iter()
            .flatten()
            .all(|source| source.step_index == 3));
        assert!(sources.iter().flatten().any(|source| source.fraction > 0.6));
        assert!(sources.iter().flatten().any(|source| source.fraction < 0.3));
        assert!(sources
            .iter()
            .zip(&mesh.machined)
            .all(|(source, cut)| *cut || source.is_none()));
        // Leading-edge contact at x=10, y=5 occurs at axis x=8.
        let cut = &field.cuts[0];
        assert!((cut.surface_location([10., 5., -2.]).unwrap().fraction - 12. / 28.).abs() < 1e-10);
    }

    #[test]
    fn same_operation_wall_bands_keep_their_separate_cutting_moves() {
        let mut field = Heightfield::new([0., 0.], [20., 10.], 0.5, 0.);
        field.set_surface_tracking(true);
        for (depth, step) in [(-2., 10), (-4., 20)] {
            field.set_cut_source(step, (0., 1.));
            field.cut_segment_profile_controlled(
                [4., 5., depth],
                [16., 5., depth],
                &ToolProfile::flat(2.),
                Some(7),
                &|| false,
            );
        }
        let mesh = field.to_solid_mesh(-6.);
        let sources = mesh.cut_sources.as_ref().unwrap();
        let mut bands = [0; 2];
        for triangle in mesh.indices.as_chunks::<3>().0 {
            let first = triangle[0] as usize;
            if mesh.machined[first] && mesh.normals[first][2].abs() < 0.1 {
                let z = triangle
                    .iter()
                    .map(|&i| mesh.positions[i as usize][2])
                    .sum::<f32>()
                    / 3.;
                let (band, step) = if z > -2. { (0, 10) } else { (1, 20) };
                bands[band] += 1;
                assert!(triangle
                    .iter()
                    .all(|&i| sources[i as usize].unwrap().step_index == step));
            }
        }
        assert!(bands.iter().all(|&n| n > 0));
    }

    #[test]
    fn thread_flanks_keep_the_cut_that_exposed_them() {
        let mut field = Heightfield::new([0., 0.], [10., 10.], 0.25, 0.);
        field.set_surface_tracking(true);
        let tool = ToolProfile {
            radius: 2.,
            shape: crate::ProfileShape::Thread {
                root_radius: 1.,
                tooth_height: 1.,
                pitch: 2.,
                tooth_count: 2,
            },
        };
        for step in [10, 20] {
            field.set_cut_source(step, (0., 1.));
            field.cut_segment_profile_controlled(
                [3., 5., -4.],
                [7., 5., -4.],
                &tool,
                Some(7),
                &|| false,
            );
        }
        let mesh = field.to_solid_mesh(-5.);
        let sources = mesh.cut_sources.as_ref().unwrap();
        assert!(sources.iter().flatten().count() > 0);
        assert!(sources.iter().flatten().all(|s| s.step_index == 10));
    }

    #[test]
    fn surface_tracking_cache_resets_on_rewind_and_mode_changes() {
        use cam_cldata::{MoveKind, Point3, ProgramBuilder};
        let program = ProgramBuilder::new()
            .feed(600.)
            .rapid(Point3::new(-4., 5., 5.), MoveKind::Link)
            .linear(Point3::new(-4., 5., -2.), MoveKind::Plunge)
            .linear(Point3::new(24., 5., -2.), MoveKind::Cutting)
            .linear(Point3::new(24., 5., -4.), MoveKind::Plunge)
            .linear(Point3::new(-4., 5., -4.), MoveKind::Cutting)
            .build();
        let mut cache = crate::SimulationCache::default();
        cache.set_surface_sources(true);
        let options = crate::SimOptions {
            resolution: 0.5,
            tool_radius: 2.,
        };
        let run = |cache: &mut crate::SimulationCache, program: &cam_cldata::Program| {
            cache
                .simulate_controlled(
                    program,
                    [0., 0., -5.],
                    [20., 10., 0.],
                    &options,
                    &[],
                    &|| false,
                    |_, _| {},
                )
                .unwrap()
                .field
                .to_solid_mesh(-5.)
        };
        let full = run(&mut cache, &program);
        assert!(full
            .cut_sources
            .unwrap()
            .iter()
            .flatten()
            .any(|s| s.step_index == 4));
        let prefix = cam_cldata::Program {
            steps: program.steps()[..3].to_vec(),
        };
        let rewound = run(&mut cache, &prefix);
        assert!(rewound
            .cut_sources
            .as_ref()
            .unwrap()
            .iter()
            .flatten()
            .all(|s| s.step_index < 3));
        let mut fresh = crate::SimulationCache::default();
        fresh.set_surface_sources(true);
        assert_eq!(rewound, run(&mut fresh, &prefix));
        cache.set_surface_sources(false);
        assert!(run(&mut cache, &prefix).cut_sources.is_none());
        cache.set_surface_sources(true);
        assert_eq!(rewound, run(&mut cache, &prefix));
    }

    #[test]
    fn coplanar_surfaces_do_not_merge_across_operation_boundaries() {
        let mut field = Heightfield::new([0., 0.], [20., 10.], 1., 0.);
        for (x, op) in [(0., 10), (20., 20)] {
            field.cut_segment_profile_controlled(
                [x, 0., -2.],
                [x, 10., -2.],
                &ToolProfile::flat(10.),
                Some(op),
                &|| false,
            );
        }
        let mesh = field.to_solid_mesh(-5.);
        assert_eq!(mesh.operations.len(), mesh.positions.len());
        assert!(mesh.operations.contains(&Some(10)) && mesh.operations.contains(&Some(20)));
        for triangle in mesh.indices.as_chunks::<3>().0 {
            let i = triangle[0] as usize;
            if mesh.normals[i][2] > 0.9 {
                let x = triangle
                    .iter()
                    .map(|&id| mesh.positions[id as usize][0])
                    .sum::<f32>()
                    / 3.;
                let expected = if x < 10. { Some(10) } else { Some(20) };
                for &id in triangle {
                    assert_eq!(mesh.operations[id as usize], expected);
                }
            } else {
                assert!(triangle
                    .iter()
                    .all(|&id| mesh.operations[id as usize].is_none()));
            }
        }
    }

    #[test]
    fn cut_walls_belong_to_the_operation_that_removed_the_neighbour() {
        let mut field = Heightfield::new([0., 0.], [20., 10.], 0.5, 0.);
        field.cut_segment_profile_controlled(
            [0., 5., -2.],
            [20., 5., -2.],
            &ToolProfile::flat(20.),
            Some(10),
            &|| false,
        );
        field.cut_segment_profile_controlled(
            [4., 5., -4.],
            [16., 5., -4.],
            &ToolProfile::flat(2.),
            Some(20),
            &|| false,
        );
        let mesh = field.to_solid_mesh(-6.);
        let mut walls = 0;
        for (i, &cut) in mesh.machined.iter().enumerate() {
            if cut && mesh.normals[i][2].abs() < 0.1 {
                walls += 1;
                assert_eq!(mesh.operations[i], Some(20));
            }
        }
        assert!(walls > 0);
    }

    #[test]
    fn deepening_a_pocket_preserves_the_previous_operations_upper_wall_band() {
        let mut field = Heightfield::new([0., 0.], [20., 10.], 0.5, 0.);
        for (depth, op) in [(-2., 10), (-4., 20)] {
            field.cut_segment_profile_controlled(
                [4., 5., depth],
                [16., 5., depth],
                &ToolProfile::flat(2.),
                Some(op),
                &|| false,
            );
        }
        let mesh = field.to_solid_mesh(-6.);
        let mut bands = [0; 2];
        for triangle in mesh.indices.as_chunks::<3>().0 {
            let i = triangle[0] as usize;
            if mesh.machined[i] && mesh.normals[i][2].abs() < 0.1 {
                let z = triangle
                    .iter()
                    .map(|&id| mesh.positions[id as usize][2])
                    .sum::<f32>()
                    / 3.;
                let (band, expected) = if z > -2. { (0, 10) } else { (1, 20) };
                bands[band] += 1;
                assert!(triangle
                    .iter()
                    .all(|&id| mesh.operations[id as usize] == Some(expected)));
            }
        }
        assert!(bands.iter().all(|&count| count > 0));
    }
}
