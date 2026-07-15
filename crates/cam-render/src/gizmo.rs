//! The orientation-cube gizmo: geometry, face labels, and a TrueType-font atlas.
//!
//! A small cube drawn in the viewport corner, rotated by the same camera as the
//! part, so the operator can always read which side they are looking at. Each
//! face is coloured by its axis and carries the **full word** for that side —
//! `Top`, `Bottom`, `Front`, `Back`, `Left`, `Right` — rendered from a real
//! **TrueType font** (Quicksand Bold, via `ab_glyph`) into a texture atlas.
//!
//! | face  | axis | word   | colour |
//! |-------|------|--------|--------|
//! | Top   | +Z   | TOP    | blue   |
//! | Bottom| −Z   | BOTTOM | dark blue |
//! | Front | −Y   | FRONT  | yellow |
//! | Back  | +Y   | BACK   | teal   |
//! | Left  | −X   | LEFT   | dark red |
//! | Right | +X   | RIGHT  | red    |
//!
//! The Y axis is **not green**: red↔green is the classic confusable pair, so a
//! green Front (−Y) and red Left (−X) read alike under red-green colour
//! deficiency. Front is **yellow** and Back **teal** — both sit clear of red for
//! that deficiency, so every face stays distinct (this axis trades one-hue-per-
//! axis for two readily separable faces).
//!
//! Rendering it (the `gpu` feature's `GizmoRenderer`) samples [`label_atlas`];
//! clicking a face snaps the view to it ([`pick_face`]). The geometry, UVs,
//! atlas, and picking here are pure and unit-tested.

/// A gizmo-cube vertex: position, outward normal, face colour, and atlas UV.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "gpu", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub struct GizmoVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 3],
    pub uv: [f32; 2],
}

/// One cube face: outward normal, colour, in-plane up direction (for label
/// orientation), and four corners CCW as seen from outside.
type Face = ([f32; 3], [f32; 3], [f32; 3], [[f32; 3]; 4]);

/// The six faces, in atlas-cell order (their labels are `label_atlas`'s cells
/// 0..6): Right, Left, Back, Front, Up, Down.
fn faces() -> [Face; 6] {
    [
        // +X Right — red, up = +Z
        (
            [1.0, 0.0, 0.0],
            [0.75, 0.20, 0.20],
            [0.0, 0.0, 1.0],
            [
                [1.0, -1.0, -1.0],
                [1.0, 1.0, -1.0],
                [1.0, 1.0, 1.0],
                [1.0, -1.0, 1.0],
            ],
        ),
        // -X Left — dark red, up = +Z
        (
            [-1.0, 0.0, 0.0],
            [0.52, 0.15, 0.15],
            [0.0, 0.0, 1.0],
            [
                [-1.0, -1.0, -1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, 1.0, 1.0],
                [-1.0, 1.0, -1.0],
            ],
        ),
        // +Y Back — teal, up = +Z
        (
            [0.0, 1.0, 0.0],
            [0.10, 0.60, 0.62],
            [0.0, 0.0, 1.0],
            [
                [-1.0, 1.0, -1.0],
                [-1.0, 1.0, 1.0],
                [1.0, 1.0, 1.0],
                [1.0, 1.0, -1.0],
            ],
        ),
        // -Y Front — bright yellow, up = +Z
        (
            [0.0, -1.0, 0.0],
            [1.0, 0.90, 0.10],
            [0.0, 0.0, 1.0],
            [
                [-1.0, -1.0, -1.0],
                [1.0, -1.0, -1.0],
                [1.0, -1.0, 1.0],
                [-1.0, -1.0, 1.0],
            ],
        ),
        // +Z Up (top) — blue, up = +Y
        (
            [0.0, 0.0, 1.0],
            [0.26, 0.42, 0.75],
            [0.0, 1.0, 0.0],
            [
                [-1.0, -1.0, 1.0],
                [1.0, -1.0, 1.0],
                [1.0, 1.0, 1.0],
                [-1.0, 1.0, 1.0],
            ],
        ),
        // -Z Down (bottom) — dark blue, up = +Y
        (
            [0.0, 0.0, -1.0],
            [0.20, 0.28, 0.48],
            [0.0, 1.0, 0.0],
            [
                [-1.0, -1.0, -1.0],
                [-1.0, 1.0, -1.0],
                [1.0, 1.0, -1.0],
                [1.0, -1.0, -1.0],
            ],
        ),
    ]
}

/// Atlas cell grid.
const ATLAS_COLS: usize = 3;
const ATLAS_ROWS: usize = 2;

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(v: [f32; 3]) -> [f32; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt().max(1e-9);
    [v[0] / l, v[1] / l, v[2] / l]
}

/// How far the edges are chamfered (fraction of the half-extent).
const CHAMFER: f32 = 0.14;
/// Neutral colour for the chamfer bevels and corners.
const EDGE_COLOR: [f32; 3] = [0.30, 0.31, 0.36];
/// UV sentinel marking a vertex that carries no label (bevels, corners).
const NO_LABEL: [f32; 2] = [-1.0, -1.0];

/// Build the orientation cube with **chamfered edges**: six inset, labelled
/// faces, twelve edge bevels, and eight corner triangles. Culling is off in the
/// renderer, so only the (explicit) normals matter, not winding.
pub fn unit_cube() -> (Vec<GizmoVertex>, Vec<u32>) {
    let inset = 1.0 - CHAMFER;
    let mut verts: Vec<GizmoVertex> = Vec::new();
    let mut idx: Vec<u32> = Vec::new();

    let mut quad = |vs: [GizmoVertex; 4]| {
        let b = verts.len() as u32;
        verts.extend_from_slice(&vs);
        idx.extend_from_slice(&[b, b + 1, b + 2, b, b + 2, b + 3]);
    };

    // Six inset, labelled faces.
    for (i, (normal, color, up, _)) in faces().into_iter().enumerate() {
        let right = cross(up, normal);
        let (col, row) = (i % ATLAS_COLS, i / ATLAS_COLS);
        let (cw, ch) = (1.0 / ATLAS_COLS as f32, 1.0 / ATLAS_ROWS as f32);
        let corner = |ru: f32, uu: f32| {
            let position = [
                normal[0] + right[0] * ru * inset + up[0] * uu * inset,
                normal[1] + right[1] * ru * inset + up[1] * uu * inset,
                normal[2] + right[2] * ru * inset + up[2] * uu * inset,
            ];
            let lu = 0.5 + 0.5 * ru * inset;
            let lv = 0.5 - 0.5 * uu * inset;
            GizmoVertex {
                position,
                normal,
                color,
                uv: [(col as f32 + lu) * cw, (row as f32 + lv) * ch],
            }
        };
        quad([
            corner(-1.0, -1.0),
            corner(1.0, -1.0),
            corner(1.0, 1.0),
            corner(-1.0, 1.0),
        ]);
    }

    // A point on axis `a`=`av`, `b`=`bv`, third axis `d`=`dv`.
    let axpt = |a: usize, av: f32, b: usize, bv: f32, d: usize, dv: f32| {
        let mut p = [0.0f32; 3];
        p[a] = av;
        p[b] = bv;
        p[d] = dv;
        p
    };
    let bevel = |position, normal| GizmoVertex {
        position,
        normal,
        color: EDGE_COLOR,
        uv: NO_LABEL,
    };

    // Twelve edge bevels: each joins the inset edges of two adjacent faces.
    for &(a, b) in &[(0usize, 1usize), (0, 2), (1, 2)] {
        let d = 3 - a - b;
        for &sa in &[-1.0f32, 1.0] {
            for &sb in &[-1.0f32, 1.0] {
                let mut n = [0.0f32; 3];
                n[a] = sa;
                n[b] = sb;
                let n = normalize(n);
                quad([
                    bevel(axpt(a, sa, b, sb * inset, d, -inset), n),
                    bevel(axpt(a, sa, b, sb * inset, d, inset), n),
                    bevel(axpt(a, sa * inset, b, sb, d, inset), n),
                    bevel(axpt(a, sa * inset, b, sb, d, -inset), n),
                ]);
            }
        }
    }

    // Eight corner triangles.
    for &sx in &[-1.0f32, 1.0] {
        for &sy in &[-1.0f32, 1.0] {
            for &sz in &[-1.0f32, 1.0] {
                let n = normalize([sx, sy, sz]);
                let b = verts.len() as u32;
                verts.push(bevel([sx, sy * inset, sz * inset], n));
                verts.push(bevel([sx * inset, sy, sz * inset], n));
                verts.push(bevel([sx * inset, sy * inset, sz], n));
                idx.extend_from_slice(&[b, b + 1, b + 2]);
            }
        }
    }

    (verts, idx)
}

// --- Face labels (rendered from a real TrueType font) ----------------------

/// The word labelling each face, in the same order as [`unit_cube`]'s faces
/// (Right, Left, Back, Front, Top, Bottom).
#[cfg(feature = "gpu")]
const FACE_WORDS: [&str; 6] = ["RIGHT", "LEFT", "BACK", "FRONT", "TOP", "BOTTOM"];

/// Atlas cell size (square, so face UVs are not stretched).
#[cfg(feature = "gpu")]
const CELL: usize = 128;

/// The embedded label font: Quicksand Bold (SIL Open Font License 1.1 — see
/// `assets/Quicksand-LICENSE.txt`).
#[cfg(feature = "gpu")]
const LABEL_FONT: &[u8] = include_bytes!("../assets/Quicksand-Bold.ttf");

/// Rasterise the six face words into an `RGBA8` atlas (a 3x2 grid of square
/// cells) from the embedded TrueType font: antialiased white text (coverage in
/// every channel), transparent elsewhere. Returns `(pixels, width, height)`.
#[cfg(feature = "gpu")]
pub fn label_atlas() -> (Vec<u8>, u32, u32) {
    use ab_glyph::{point, Font, FontRef, PxScale, ScaleFont};

    let font = FontRef::try_from_slice(LABEL_FONT).expect("valid embedded font");
    let (w, h) = (CELL * ATLAS_COLS, CELL * ATLAS_ROWS);
    let mut px = vec![0u8; w * h * 4];

    let word_width = |word: &str, scale: f32| -> f32 {
        let sf = font.as_scaled(PxScale::from(scale));
        word.chars().map(|c| sf.h_advance(font.glyph_id(c))).sum()
    };

    for (i, &word) in FACE_WORDS.iter().enumerate() {
        let (col, row) = (i % ATLAS_COLS, i / ATLAS_COLS);
        // Shrink the scale until the word fits ~84% of the cell width.
        let mut scale = CELL as f32 * 0.5;
        while word_width(word, scale) > CELL as f32 * 0.84 && scale > 6.0 {
            scale *= 0.92;
        }
        let sf = font.as_scaled(PxScale::from(scale));
        let text_w = word_width(word, scale);
        let text_h = sf.ascent() - sf.descent();
        let cell_x = (col * CELL) as f32;
        let cell_y = (row * CELL) as f32;
        let mut pen_x = cell_x + (CELL as f32 - text_w) / 2.0;
        let baseline = cell_y + (CELL as f32 - text_h) / 2.0 + sf.ascent();

        for c in word.chars() {
            let id = font.glyph_id(c);
            let glyph = id.with_scale_and_position(PxScale::from(scale), point(pen_x, baseline));
            if let Some(outline) = font.outline_glyph(glyph) {
                let bounds = outline.px_bounds();
                outline.draw(|gx, gy, coverage| {
                    let x = bounds.min.x as i32 + gx as i32;
                    let y = bounds.min.y as i32 + gy as i32;
                    if x < 0 || y < 0 || x as usize >= w || y as usize >= h {
                        return;
                    }
                    let idx = (y as usize * w + x as usize) * 4;
                    let v = (coverage * 255.0) as u8;
                    if v > px[idx] {
                        px[idx..idx + 4].copy_from_slice(&[v, v, v, 255]);
                    }
                });
            }
            pen_x += sf.h_advance(id);
        }
    }
    (px, w as u32, h as u32)
}

/// The face (as an outward unit normal) hit by a click at gizmo NDC `(u, v)`
/// (each in `-1..1`), under the given `orient` — or `None` if the click misses
/// the cube. Used to make the gizmo faces clickable. `half` is the orthographic
/// half-extent the cube is framed in (so `u,v = ±1` reach the view edges).
pub fn pick_face(orient: [[f32; 4]; 4], half: f32, u: f32, v: f32) -> Option<[f32; 3]> {
    // World axes of the view (rows of the world→view rotation).
    let right = [orient[0][0], orient[1][0], orient[2][0]];
    let up = [orient[0][1], orient[1][1], orient[2][1]];
    let fwd = [orient[0][2], orient[1][2], orient[2][2]]; // toward viewer
                                                          // Orthographic ray: start on the view plane at (u,v), aim into the scene.
    let origin = [
        right[0] * u * half + up[0] * v * half + fwd[0] * 4.0,
        right[1] * u * half + up[1] * v * half + fwd[1] * 4.0,
        right[2] * u * half + up[2] * v * half + fwd[2] * 4.0,
    ];
    let dir = [-fwd[0], -fwd[1], -fwd[2]];
    ray_unit_cube_face(origin, dir)
}

/// Ray vs. the axis-aligned unit cube `[-1,1]³`: the outward normal of the face
/// first entered, or `None` if missed.
fn ray_unit_cube_face(o: [f32; 3], d: [f32; 3]) -> Option<[f32; 3]> {
    let mut t_near = f32::MIN;
    let mut axis = 0usize;
    let mut sign = 0.0f32;
    for a in 0..3 {
        if d[a].abs() < 1e-9 {
            if o[a] < -1.0 || o[a] > 1.0 {
                return None; // parallel and outside the slab
            }
            continue;
        }
        let inv = 1.0 / d[a];
        let mut t1 = (-1.0 - o[a]) * inv;
        let mut t2 = (1.0 - o[a]) * inv;
        let mut s = -1.0;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
            s = 1.0;
        }
        if t1 > t_near {
            t_near = t1;
            axis = a;
            sign = s;
        }
        if t_near > t2 {
            return None; // slabs don't overlap ⇒ miss
        }
    }
    if t_near <= f32::MIN {
        return None;
    }
    let mut n = [0.0, 0.0, 0.0];
    n[axis] = sign;
    Some(n)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chamfered_cube_has_six_labelled_faces() {
        let (v, i) = unit_cube();
        // 6 face quads (24) + 12 bevel quads (48) + 8 corner tris (24) = 96;
        // indices 36 + 72 + 24 = 132.
        assert_eq!(v.len(), 96, "faces + bevels + corners");
        assert_eq!(i.len(), 132);
        assert!(i.iter().all(|&idx| (idx as usize) < v.len()));

        // The first 24 vertices are the six labelled faces (one axis-aligned,
        // distinctly coloured, unit-normal face per group of four).
        let mut colors = Vec::new();
        for face in v[..24].chunks(4) {
            let n = face[0].normal;
            let c = face[0].color;
            assert!(face.iter().all(|x| x.normal == n && x.color == c));
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            assert!((len - 1.0).abs() < 1e-6, "unit normal");
            colors.push(c);
        }
        for a in 0..colors.len() {
            for b in (a + 1)..colors.len() {
                assert_ne!(colors[a], colors[b], "faces {a},{b} share a colour");
            }
        }
        // Bevel/corner vertices carry the no-label sentinel; face vertices don't.
        assert!(v[..24].iter().all(|x| x.uv[0] >= 0.0), "faces labelled");
        assert!(v[24..].iter().all(|x| x.uv == NO_LABEL), "edges unlabelled");
    }

    #[test]
    fn each_face_uses_its_own_atlas_cell() {
        // Each labelled face's UVs sit within a distinct cell (identified by the
        // centroid), and the label fills the inset portion of the cell.
        let (v, _) = unit_cube();
        let mut cells = std::collections::BTreeSet::new();
        for face in v[..24].chunks(4) {
            let cu = face.iter().map(|x| x.uv[0]).sum::<f32>() / 4.0;
            let cv = face.iter().map(|x| x.uv[1]).sum::<f32>() / 4.0;
            let col = (cu * ATLAS_COLS as f32).floor() as usize;
            let row = (cv * ATLAS_ROWS as f32).floor() as usize;
            let umin = face.iter().map(|x| x.uv[0]).fold(f32::MAX, f32::min);
            let umax = face.iter().map(|x| x.uv[0]).fold(f32::MIN, f32::max);
            let expected = (1.0 - CHAMFER) / ATLAS_COLS as f32;
            assert!((umax - umin - expected).abs() < 1e-5, "label fills inset");
            assert!(cells.insert((col, row)), "faces share a cell");
        }
        assert_eq!(cells.len(), 6);
    }

    #[cfg(feature = "gpu")]
    #[test]
    fn atlas_has_ink_for_every_word() {
        let (px, w, h) = label_atlas();
        assert_eq!(px.len(), (w * h * 4) as usize);
        // Each of the six cells must contain rendered (white) text.
        for (i, &word) in FACE_WORDS.iter().enumerate() {
            let (col, row) = (i % ATLAS_COLS, i / ATLAS_COLS);
            let mut ink = 0;
            for y in (row * CELL)..((row + 1) * CELL) {
                for x in (col * CELL)..((col + 1) * CELL) {
                    if px[(y * w as usize + x) * 4] > 0 {
                        ink += 1;
                    }
                }
            }
            assert!(ink > 30, "cell {i} ({word}) has too little ink: {ink}");
        }
    }

    #[test]
    fn clicking_a_face_picks_it() {
        use crate::orientation;
        let half = 3f32.sqrt() * 1.1; // unit-cube framing (radius·(1+margin))
                                      // Top view: a centre click hits the top face (+Z).
        assert_eq!(
            pick_face(orientation(0.0, 0.0), half, 0.0, 0.0),
            Some([0.0, 0.0, 1.0])
        );
        // pitch 90° faces the +Y side (per the camera convention): centre click
        // picks +Y.
        let n = pick_face(
            orientation(0.0, std::f32::consts::FRAC_PI_2),
            half,
            0.0,
            0.0,
        )
        .unwrap();
        assert!(n[1] > 0.5, "pitch 90° centre picks +Y, got {n:?}");
        // A click far outside the cube misses.
        assert_eq!(pick_face(orientation(0.0, 0.0), half, 0.99, 0.99), None);
    }
}
