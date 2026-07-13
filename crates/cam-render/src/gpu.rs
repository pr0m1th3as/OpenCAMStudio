//! The `wgpu` renderers.
//!
//! [`LineRenderer`] draws the [`Scene`](crate::Scene)'s [`Vertex`]es as a
//! `LineList` (part outline + backplot); [`MeshRenderer`] draws a solid stock
//! surface ([`MeshVertex`]es + indices) with normal-based shading. Both take a
//! camera uniform and record into a caller-supplied render pass — the caller
//! owns the `wgpu` device/queue and pass. Only built with the `gpu` feature.

use wgpu::util::DeviceExt;

use crate::mesh::MeshVertex;
use crate::scene::Vertex;

/// A camera uniform buffer (one `mat4x4<f32>`), its bind-group layout, and the
/// matching bind group — shared setup for both renderers.
struct Camera {
    buffer: wgpu::Buffer,
    layout: wgpu::BindGroupLayout,
    bind_group: wgpu::BindGroup,
}

impl Camera {
    fn new(device: &wgpu::Device) -> Self {
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("cam-render camera"),
            size: 64, // one mat4x4<f32>
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("cam-render camera layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("cam-render camera bind group"),
            layout: &layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: buffer.as_entire_binding(),
            }],
        });
        Self {
            buffer,
            layout,
            bind_group,
        }
    }

    fn set(&self, queue: &wgpu::Queue, view_proj: [[f32; 4]; 4]) {
        queue.write_buffer(&self.buffer, 0, bytemuck::cast_slice(&[view_proj]));
    }
}

const SHADER: &str = r#"
struct Camera { view_proj: mat4x4<f32> };
@group(0) @binding(0) var<uniform> camera: Camera;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(@location(0) position: vec3<f32>, @location(1) color: vec4<f32>) -> VsOut {
    var out: VsOut;
    out.clip = camera.view_proj * vec4<f32>(position, 1.0);
    out.color = color;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    return in.color;
}
"#;

/// Draws scene line segments with `wgpu`.
pub struct LineRenderer {
    pipeline: wgpu::RenderPipeline,
    camera: Camera,
    vertex_buffer: Option<wgpu::Buffer>,
    vertex_count: u32,
}

impl LineRenderer {
    /// Create the renderer for a surface of the given `format`.
    pub fn new(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cam-render line shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });

        let camera = Camera::new(device);

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cam-render pipeline layout"),
            bind_group_layouts: &[&camera.layout],
            push_constant_ranges: &[],
        });

        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: 0,
                    shader_location: 0,
                },
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x4,
                    offset: 12,
                    shader_location: 1,
                },
            ],
        };

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cam-render line pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[vertex_layout],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self {
            pipeline,
            camera,
            vertex_buffer: None,
            vertex_count: 0,
        }
    }

    /// Upload the camera view-projection (column-major).
    pub fn set_camera(&self, queue: &wgpu::Queue, view_proj: [[f32; 4]; 4]) {
        self.camera.set(queue, view_proj);
    }

    /// Upload the scene's line vertices, replacing any previous set.
    pub fn upload(&mut self, device: &wgpu::Device, vertices: &[Vertex]) {
        self.vertex_count = vertices.len() as u32;
        self.vertex_buffer = if vertices.is_empty() {
            None
        } else {
            Some(
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("cam-render vertices"),
                    contents: bytemuck::cast_slice(vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                }),
            )
        };
    }

    /// Record draw commands into an existing render pass.
    pub fn draw(&self, pass: &mut wgpu::RenderPass<'_>) {
        let Some(buffer) = &self.vertex_buffer else {
            return;
        };
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.camera.bind_group, &[]);
        pass.set_vertex_buffer(0, buffer.slice(..));
        pass.draw(0..self.vertex_count, 0..1);
    }
}

const MESH_SHADER: &str = r#"
struct Camera { view_proj: mat4x4<f32> };
@group(0) @binding(0) var<uniform> camera: Camera;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
};

@vertex
fn vs_main(@location(0) position: vec3<f32>, @location(1) normal: vec3<f32>) -> VsOut {
    var out: VsOut;
    out.clip = camera.view_proj * vec4<f32>(position, 1.0);
    out.normal = normal;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    // A fixed key light from the upper front; world-space normals suffice under
    // the orthographic top view (no non-uniform transform). Ambient keeps floors
    // lit while the diffuse term reveals pocket walls and steps.
    let light = normalize(vec3<f32>(0.35, 0.45, 0.82));
    let n = normalize(in.normal);
    let lambert = max(dot(n, light), 0.0);
    let base = vec3<f32>(0.55, 0.58, 0.63); // machined-steel grey
    let shade = base * (0.35 + 0.65 * lambert);
    return vec4<f32>(shade, 1.0);
}
"#;

/// Draws a solid stock surface with `wgpu`: indexed triangles shaded by their
/// surface normal.
///
/// It carries no depth buffer, which is correct for the viewport's orthographic
/// **top** view — a heightfield `z(x, y)` seen straight down never occludes
/// itself. A tilted or perspective camera would need a depth attachment added
/// here.
pub struct MeshRenderer {
    pipeline: wgpu::RenderPipeline,
    camera: Camera,
    vertex_buffer: Option<wgpu::Buffer>,
    index_buffer: Option<wgpu::Buffer>,
    index_count: u32,
}

impl MeshRenderer {
    /// Create the renderer for a surface of the given `format`.
    pub fn new(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cam-render mesh shader"),
            source: wgpu::ShaderSource::Wgsl(MESH_SHADER.into()),
        });

        let camera = Camera::new(device);

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cam-render mesh pipeline layout"),
            bind_group_layouts: &[&camera.layout],
            push_constant_ranges: &[],
        });

        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<MeshVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: 0,
                    shader_location: 0,
                },
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: 12,
                    shader_location: 1,
                },
            ],
        };

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cam-render mesh pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[vertex_layout],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self {
            pipeline,
            camera,
            vertex_buffer: None,
            index_buffer: None,
            index_count: 0,
        }
    }

    /// Upload the camera view-projection (column-major).
    pub fn set_camera(&self, queue: &wgpu::Queue, view_proj: [[f32; 4]; 4]) {
        self.camera.set(queue, view_proj);
    }

    /// Upload the mesh (interleaved position+normal vertices and triangle
    /// indices), replacing any previous mesh. An empty mesh clears the surface.
    pub fn upload(&mut self, device: &wgpu::Device, vertices: &[MeshVertex], indices: &[u32]) {
        self.index_count = indices.len() as u32;
        if vertices.is_empty() || indices.is_empty() {
            self.vertex_buffer = None;
            self.index_buffer = None;
            return;
        }
        self.vertex_buffer = Some(
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cam-render mesh vertices"),
                contents: bytemuck::cast_slice(vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }),
        );
        self.index_buffer = Some(
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cam-render mesh indices"),
                contents: bytemuck::cast_slice(indices),
                usage: wgpu::BufferUsages::INDEX,
            }),
        );
    }

    /// Record draw commands into an existing render pass.
    pub fn draw(&self, pass: &mut wgpu::RenderPass<'_>) {
        let (Some(vertices), Some(indices)) = (&self.vertex_buffer, &self.index_buffer) else {
            return;
        };
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.camera.bind_group, &[]);
        pass.set_vertex_buffer(0, vertices.slice(..));
        pass.set_index_buffer(indices.slice(..), wgpu::IndexFormat::Uint32);
        pass.draw_indexed(0..self.index_count, 0, 0..1);
    }
}
