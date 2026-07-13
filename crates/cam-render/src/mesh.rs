//! Solid-surface mesh data for the viewport: interleaved position+normal
//! vertices and the pure mapping that builds them. Like [`Scene`](crate::Scene),
//! nothing here touches a GPU, so it is unit-tested with the rest of the
//! pipeline; the `gpu` feature's [`MeshRenderer`](crate::MeshRenderer) uploads
//! and draws these.
//!
//! The mesh itself (a triangulated stock heightfield) is produced by `cam-sim`
//! as parallel position/normal/index arrays. We deliberately take those as raw
//! slices rather than depend on `cam-sim`: the renderer draws surfaces, it need
//! not know they came from a simulation.

/// A vertex for solid surface rendering: position (mm) and unit normal. Plain
/// data; the `gpu` feature makes it `bytemuck`-castable for upload.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "gpu", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub struct MeshVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

/// Interleave parallel `positions` and `normals` (as produced by `cam-sim`'s
/// `SurfaceMesh`) into [`MeshVertex`]es. The two slices are expected to be the
/// same length — one normal per position; any surplus tail on the longer slice
/// is ignored.
pub fn mesh_vertices(positions: &[[f32; 3]], normals: &[[f32; 3]]) -> Vec<MeshVertex> {
    positions
        .iter()
        .zip(normals)
        .map(|(&position, &normal)| MeshVertex { position, normal })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mesh_vertices_interleaves_position_and_normal() {
        let pos = [[0.0, 0.0, -1.0], [1.0, 0.0, -1.0]];
        let nrm = [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]];
        let v = mesh_vertices(&pos, &nrm);
        assert_eq!(v.len(), 2);
        assert_eq!(v[0].position, [0.0, 0.0, -1.0]);
        assert_eq!(v[1].normal, [0.0, 0.0, 1.0]);
    }

    #[test]
    fn mismatched_lengths_stop_at_the_shorter() {
        let pos = [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]];
        let nrm = [[0.0, 0.0, 1.0]];
        assert_eq!(mesh_vertices(&pos, &nrm).len(), 1);
    }
}
