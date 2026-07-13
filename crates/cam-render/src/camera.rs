//! Camera math for the viewport — a pure, testable orbit camera.
//!
//! Matrices are column-major `[[f32; 4]; 4]` (the layout WGSL expects) and map
//! world millimetres into wgpu clip space (x,y ∈ [-1, 1], z ∈ [0, 1], y up).
//!
//! The interactive view is an **orthographic orbit** ([`OrbitCamera`]): the
//! stock is framed by its bounding sphere so it never clips at any angle, and
//! `yaw = pitch = 0` is the top-down view (matching [`top_view`]). Orthographic,
//! not perspective, is deliberate — a CAM viewport must not foreshorten, so what
//! you measure on screen is true. [`top_view`] remains as the plain matrix used
//! for the initial framing and by tests.

/// Column-major 4×4 matrix (a set of four columns).
type Mat4 = [[f32; 4]; 4];

/// Extra fraction of empty space framed around the geometry.
const MARGIN: f32 = 0.1;

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

/// An orthographic orbit camera around a target point.
///
/// Orientation is a free **world→view rotation** matrix (`orient`) — not
/// Euler angles — so the view tumbles to *any* orientation, including the
/// underside, with no clamp or gimbal lock. Identity is the top-down view
/// (matching [`top_view`]). `zoom` magnifies (a factor of `2^zoom`); `pan` shifts
/// the target in world space. The scene is framed by its bounding sphere so it
/// stays fully visible at any orientation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrbitCamera {
    /// Point the camera orbits, before panning (world mm).
    pub target: [f32; 3],
    /// Bounding-sphere radius of the framed scene (world mm).
    pub radius: f32,
    /// World→view rotation (column-major; upper-left 3×3 is orthonormal).
    pub orient: Mat4,
    /// Magnification exponent: on-screen scale is `2^zoom`.
    pub zoom: f32,
    /// Target shift in world space (mm), accumulated by panning.
    pub pan: [f32; 3],
}

impl OrbitCamera {
    /// Frame the world box `(min, max)` top-down: target at its centre, radius
    /// its bounding sphere, no rotation / zoom / pan.
    pub fn framed(min: [f32; 3], max: [f32; 3]) -> Self {
        let target = [
            0.5 * (min[0] + max[0]),
            0.5 * (min[1] + max[1]),
            0.5 * (min[2] + max[2]),
        ];
        let dx = 0.5 * (max[0] - min[0]);
        let dy = 0.5 * (max[1] - min[1]);
        let dz = 0.5 * (max[2] - min[2]);
        let radius = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        Self {
            target,
            radius,
            orient: IDENTITY,
            zoom: 0.0,
            pan: [0.0, 0.0, 0.0],
        }
    }

    /// The vertical half-extent of the orthographic view, mm (before aspect).
    pub fn half_height(&self) -> f32 {
        self.radius * (1.0 + MARGIN) / 2.0_f32.powf(self.zoom)
    }

    /// World millimetres spanned by one viewport pixel (for pan sensitivity).
    pub fn world_per_pixel(&self, viewport_height: f32) -> f32 {
        if viewport_height > 0.0 {
            2.0 * self.half_height() / viewport_height
        } else {
            1.0
        }
    }

    /// The camera's world-space right axis (screen +X).
    pub fn right(&self) -> [f32; 3] {
        [self.orient[0][0], self.orient[1][0], self.orient[2][0]]
    }

    /// The camera's world-space up axis (screen +Y).
    pub fn up(&self) -> [f32; 3] {
        [self.orient[0][1], self.orient[1][1], self.orient[2][1]]
    }

    /// The view-projection matrix for a viewport of the given `aspect`
    /// (width / height), column-major.
    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        let aspect = if aspect.is_finite() && aspect > 0.0 {
            aspect
        } else {
            1.0
        };
        let half = self.half_height();
        let (half_w, half_h) = if aspect >= 1.0 {
            (half * aspect, half)
        } else {
            (half, half / aspect)
        };

        // Push the scene in front of the camera so depths are positive; near/far
        // straddle the bounding sphere with margin.
        let dist = 2.0 * self.radius;
        let near = (dist - 1.5 * self.radius).max(1e-3);
        let far = dist + 1.5 * self.radius;

        let eff_target = [
            self.target[0] + self.pan[0],
            self.target[1] + self.pan[1],
            self.target[2] + self.pan[2],
        ];
        let view = mul(
            &translation(0.0, 0.0, -dist),
            &mul(
                &self.orient,
                &translation(-eff_target[0], -eff_target[1], -eff_target[2]),
            ),
        );
        let proj = orthographic_rh(-half_w, half_w, -half_h, half_h, near, far);
        mul(&proj, &view)
    }
}

/// The identity rotation — the top-down view.
pub const IDENTITY: Mat4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

/// The world→view rotation `R_x(pitch) · R_z(yaw)` for a turntable orientation:
/// `yaw` spins about the world up (Z) axis, `pitch` tilts from the top-down view.
/// Unclamped — `pitch` may carry all the way round to the underside — yet the
/// horizon stays level, because `yaw` is always about world up.
pub fn orientation(yaw: f32, pitch: f32) -> Mat4 {
    mul(&rotation_x(pitch), &rotation_z(yaw))
}

/// Column-major matrix product `a · b`.
fn mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut o = [[0.0f32; 4]; 4];
    for (c, oc) in o.iter_mut().enumerate() {
        for (r, ocr) in oc.iter_mut().enumerate() {
            let mut s = 0.0;
            for k in 0..4 {
                s += a[k][r] * b[c][k];
            }
            *ocr = s;
        }
    }
    o
}

/// Translation matrix.
fn translation(x: f32, y: f32, z: f32) -> Mat4 {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [x, y, z, 1.0],
    ]
}

/// Rotation about the Z axis by `a` radians.
fn rotation_z(a: f32) -> Mat4 {
    let (s, c) = a.sin_cos();
    [
        [c, s, 0.0, 0.0],
        [-s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Rotation about the X axis by `a` radians.
fn rotation_x(a: f32) -> Mat4 {
    let (s, c) = a.sin_cos();
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, s, 0.0],
        [0.0, -s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Right-handed orthographic projection into wgpu clip space (z ∈ [0, 1]), for a
/// view looking down its −Z with positive `near`/`far` distances. Column-major.
fn orthographic_rh(l: f32, r: f32, b: f32, t: f32, near: f32, far: f32) -> Mat4 {
    let rcp_w = 1.0 / (r - l);
    let rcp_h = 1.0 / (t - b);
    let rr = 1.0 / (near - far);
    [
        [2.0 * rcp_w, 0.0, 0.0, 0.0],
        [0.0, 2.0 * rcp_h, 0.0, 0.0],
        [0.0, 0.0, rr, 0.0],
        [-(l + r) * rcp_w, -(t + b) * rcp_h, rr * near, 1.0],
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

    const MIN: [f32; 3] = [0.0, 0.0, -5.0];
    const MAX: [f32; 3] = [20.0, 10.0, 0.0];
    const CORNERS: [[f32; 3]; 8] = [
        [0.0, 0.0, -5.0],
        [20.0, 0.0, -5.0],
        [20.0, 10.0, -5.0],
        [0.0, 10.0, -5.0],
        [0.0, 0.0, 0.0],
        [20.0, 0.0, 0.0],
        [20.0, 10.0, 0.0],
        [0.0, 10.0, 0.0],
    ];

    #[test]
    fn orbit_top_view_centres_the_scene() {
        let cam = OrbitCamera::framed(MIN, MAX);
        let c = transform(&cam.view_proj(1.0), cam.target);
        assert!(
            c[0].abs() < 1e-5 && c[1].abs() < 1e-5,
            "centre → origin: {c:?}"
        );
        assert!((0.0..=1.0).contains(&c[2]), "z in [0,1]: {}", c[2]);
    }

    #[test]
    fn orbit_top_view_keeps_x_right_and_y_up() {
        // At yaw=pitch=0 the view is the plain top view: +X → right, +Y → up.
        let cam = OrbitCamera::framed(MIN, MAX);
        let m = cam.view_proj(1.0);
        let right = transform(&m, [cam.target[0] + 5.0, cam.target[1], cam.target[2]]);
        let up = transform(&m, [cam.target[0], cam.target[1] + 5.0, cam.target[2]]);
        assert!(
            right[0] > 0.0 && right[1].abs() < 1e-5,
            "right maps +x: {right:?}"
        );
        assert!(up[1] > 0.0 && up[0].abs() < 1e-5, "up maps +y: {up:?}");
    }

    #[test]
    fn orbit_frames_all_corners_at_any_angle() {
        // The bounding-sphere framing must keep every corner inside clip space
        // and within the depth range, whatever the orientation.
        for &(yaw, pitch) in &[(0.0, 0.0), (0.6, 0.5), (-1.2, 0.9), (2.5, -0.7)] {
            let mut cam = OrbitCamera::framed(MIN, MAX);
            cam.orient = orientation(yaw, pitch);
            let m = cam.view_proj(16.0 / 9.0);
            for corner in CORNERS {
                let c = transform(&m, corner);
                assert!(
                    c[0].abs() <= 1.0 + 1e-4 && c[1].abs() <= 1.0 + 1e-4,
                    "corner {corner:?} → {c:?} at ({yaw},{pitch})"
                );
                assert!(
                    (-1e-4..=1.0 + 1e-4).contains(&c[2]),
                    "depth {} out of range at ({yaw},{pitch})",
                    c[2]
                );
            }
        }
    }

    #[test]
    fn orbit_right_and_up_are_orthonormal() {
        let mut cam = OrbitCamera::framed(MIN, MAX);
        cam.orient = orientation(0.8, 0.6);
        let (r, u) = (cam.right(), cam.up());
        let dot = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
        let rl = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
        let ul = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
        assert!(dot.abs() < 1e-5, "right·up = {dot}");
        assert!(
            (rl - 1.0).abs() < 1e-5 && (ul - 1.0).abs() < 1e-5,
            "unit length"
        );
    }

    #[test]
    fn underside_is_reachable_past_the_old_clamp() {
        // A turntable pitch of π (well past the old 90° clamp) looks at the
        // underside: the frame stays orthonormal, the part stays on screen, and
        // world −Z is nearer the camera than +Z.
        let mut cam = OrbitCamera::framed(MIN, MAX);
        cam.orient = orientation(0.0, std::f32::consts::PI);

        let (r, u) = (cam.right(), cam.up());
        let dot = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
        assert!(dot.abs() < 1e-4, "orthogonal at pitch π: {dot}");

        let m = cam.view_proj(1.0);
        for corner in CORNERS {
            let c = transform(&m, corner);
            assert!(
                c[0].abs() <= 1.0 + 1e-3 && c[1].abs() <= 1.0 + 1e-3,
                "framed: {c:?}"
            );
            assert!((-1e-3..=1.0 + 1e-3).contains(&c[2]), "depth {}", c[2]);
        }
        let below = transform(&m, [cam.target[0], cam.target[1], cam.target[2] - 3.0]);
        let above = transform(&m, [cam.target[0], cam.target[1], cam.target[2] + 3.0]);
        assert!(below[2] < above[2], "underside now faces the camera");
    }

    #[test]
    fn zoom_magnifies_and_pan_shifts() {
        let base = OrbitCamera::framed(MIN, MAX);
        let mut zoomed = base;
        zoomed.zoom = 1.0; // 2× magnification ⇒ half the world half-extent
        assert!((zoomed.half_height() - base.half_height() / 2.0).abs() < 1e-4);

        // Panning the target by +right moves the projected centre by −x.
        let mut panned = base;
        let r = base.right();
        let d = 3.0;
        panned.pan = [r[0] * d, r[1] * d, r[2] * d];
        let c = transform(&panned.view_proj(1.0), base.target);
        assert!(
            c[0] < -1e-3,
            "target panned right ⇒ centre left of origin: {c:?}"
        );
    }
}
