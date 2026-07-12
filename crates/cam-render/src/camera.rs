//! Camera math for the viewport — a pure, testable orthographic top-view.
//!
//! Matrices are column-major `[[f32; 4]; 4]` (the layout WGSL expects) and map
//! world millimetres into wgpu clip space (x,y ∈ [-1, 1], z ∈ [0, 1], y up).

/// Build a top-down orthographic view-projection that frames the world XY box
/// `(min, max)` inside a viewport of the given `aspect` (width / height), with a
/// fractional `margin` of empty space around the geometry.
///
/// The box is expanded to the viewport's aspect ratio so the geometry is never
/// stretched. `z` is mapped from `[min_z - 1, max_z + 1]` so the whole scene is
/// within the depth range.
pub fn top_view(min: [f32; 3], max: [f32; 3], aspect: f32, margin: f32) -> [[f32; 4]; 4] {
    let cx = 0.5 * (min[0] + max[0]);
    let cy = 0.5 * (min[1] + max[1]);
    let mut half_w = 0.5 * (max[0] - min[0]).max(1e-3) * (1.0 + margin);
    let mut half_h = 0.5 * (max[1] - min[1]).max(1e-3) * (1.0 + margin);

    // Grow the smaller axis so the world box matches the viewport aspect.
    let aspect = if aspect.is_finite() && aspect > 0.0 {
        aspect
    } else {
        1.0
    };
    if half_w / half_h < aspect {
        half_w = half_h * aspect;
    } else {
        half_h = half_w / aspect;
    }

    let (l, r) = (cx - half_w, cx + half_w);
    let (b, t) = (cy - half_h, cy + half_h);
    // Looking straight down: the highest world z is nearest (clip z 0), the
    // lowest is farthest (clip z 1).
    let z_near = max[2] + 1.0;
    let z_far = min[2] - 1.0;
    orthographic(l, r, b, t, z_near, z_far)
}

/// Right-handed orthographic projection into wgpu clip space (x,y ∈ [-1, 1],
/// z ∈ [0, 1] with `z_near` → 0 and `z_far` → 1), column-major.
fn orthographic(l: f32, r: f32, b: f32, t: f32, z_near: f32, z_far: f32) -> [[f32; 4]; 4] {
    let rl = r - l;
    let tb = t - b;
    let range = z_near - z_far;
    [
        [2.0 / rl, 0.0, 0.0, 0.0],
        [0.0, 2.0 / tb, 0.0, 0.0],
        [0.0, 0.0, -1.0 / range, 0.0],
        [-(r + l) / rl, -(t + b) / tb, z_near / range, 1.0],
    ]
}

/// Multiply a column-major matrix by a homogeneous point (for tests / picking).
#[cfg(test)]
fn transform(m: &[[f32; 4]; 4], p: [f32; 3]) -> [f32; 3] {
    let mut out = [0.0f32; 4];
    for (i, o) in out.iter_mut().enumerate() {
        *o = m[0][i] * p[0] + m[1][i] * p[1] + m[2][i] * p[2] + m[3][i];
    }
    [out[0], out[1], out[2]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centre_of_scene_maps_to_clip_origin() {
        let m = top_view([0.0, 0.0, -5.0], [20.0, 10.0, 0.0], 1.0, 0.0);
        let c = transform(&m, [10.0, 5.0, -2.5]);
        assert!(
            c[0].abs() < 1e-5 && c[1].abs() < 1e-5,
            "centre → origin, got {c:?}"
        );
    }

    #[test]
    fn geometry_fits_within_clip_bounds() {
        let m = top_view([0.0, 0.0, -5.0], [20.0, 10.0, 0.0], 2.0, 0.1);
        for corner in [[0.0, 0.0, 0.0], [20.0, 10.0, 0.0], [0.0, 10.0, -5.0]] {
            let c = transform(&m, corner);
            assert!(
                c[0].abs() <= 1.0 + 1e-4 && c[1].abs() <= 1.0 + 1e-4,
                "{corner:?} → {c:?}"
            );
            assert!((0.0..=1.0).contains(&c[2]), "z in [0,1], got {}", c[2]);
        }
    }

    #[test]
    fn wider_viewport_widens_the_world_box_not_the_geometry() {
        // With a 2:1 viewport and a square scene, the mapped x extent shrinks
        // (more world fits horizontally) while y fills.
        let m = top_view([0.0, 0.0, 0.0], [10.0, 10.0, 0.0], 2.0, 0.0);
        let right = transform(&m, [10.0, 5.0, 0.0]);
        assert!(
            right[0] < 1.0 - 1e-3,
            "square scene should not fill a wide viewport in x"
        );
    }
}
