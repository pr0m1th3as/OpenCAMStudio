//! The `wgpu` renderers.
//!
//! [`LineRenderer`] draws the [`Scene`](crate::Scene)'s [`Vertex`]es as a
//! `LineList` (part outline + backplot); [`MeshRenderer`] draws a solid stock
//! surface ([`MeshVertex`]es + indices) with normal-based shading. Both take a
//! camera uniform and record into a caller-supplied render pass — the caller
//! owns the `wgpu` device/queue and pass. Only built with the `gpu` feature.

use wgpu::util::DeviceExt;

use crate::gizmo::GizmoVertex;
use crate::mesh::MeshVertex;
use crate::scene::Vertex;

/// Depth-buffer format the renderers expect. The host must attach a depth
/// texture of this format to the render pass it hands to [`LineRenderer::draw`] /
/// [`MeshRenderer::draw`] — the orbit view relies on depth to occlude a rotated
/// solid correctly.
pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Depth state for the solid mesh: test against and write the depth buffer, so
/// nearer triangles hide farther ones from any angle.
fn mesh_depth() -> wgpu::DepthStencilState {
    wgpu::DepthStencilState {
        format: DEPTH_FORMAT,
        depth_write_enabled: true,
        depth_compare: wgpu::CompareFunction::Less,
        stencil: wgpu::StencilState::default(),
        bias: wgpu::DepthBiasState::default(),
    }
}

/// Depth state for the backplot lines: participate in the depth pass but never
/// occlude — the toolpath is drawn over the stock, never hidden inside it.
fn line_depth() -> wgpu::DepthStencilState {
    wgpu::DepthStencilState {
        format: DEPTH_FORMAT,
        depth_write_enabled: false,
        depth_compare: wgpu::CompareFunction::Always,
        stencil: wgpu::StencilState::default(),
        bias: wgpu::DepthBiasState::default(),
    }
}

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
            depth_stencil: Some(line_depth()),
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
/// surface normal, depth-tested so the rotated solid occludes itself correctly.
/// The host must attach a [`DEPTH_FORMAT`] depth texture to the pass.
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
            depth_stencil: Some(mesh_depth()),
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

const GIZMO_SHADER: &str = r#"
struct Camera { view_proj: mat4x4<f32> };
@group(0) @binding(0) var<uniform> camera: Camera;
@group(1) @binding(0) var atlas: texture_2d<f32>;
@group(1) @binding(1) var atlas_sampler: sampler;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

@vertex
fn vs_main(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) uv: vec2<f32>,
) -> VsOut {
    var out: VsOut;
    out.clip = camera.view_proj * vec4<f32>(position, 1.0);
    out.normal = normal;
    out.color = color;
    out.uv = uv;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let light = normalize(vec3<f32>(0.3, 0.4, 0.85));
    let n = normalize(in.normal);
    let lambert = max(dot(n, light), 0.0);
    let shaded = in.color * (0.45 + 0.55 * lambert);
    // White label composited over the shaded face by its atlas coverage. A
    // negative UV marks an unlabelled vertex (the chamfer bevels and corners).
    var coverage = 0.0;
    if (in.uv.x >= 0.0) {
        coverage = textureSample(atlas, atlas_sampler, in.uv).r;
    }
    let rgb = mix(shaded, vec3<f32>(1.0, 1.0, 1.0), coverage);
    return vec4<f32>(rgb, 1.0);
}
"#;

/// Draws the orientation-cube gizmo — a small cube, per-face coloured and
/// **labelled** (`U/D/F/B/L/R`), depth-tested and rotated by the viewport camera
/// so the current orientation is always legible. Geometry and the label atlas
/// are static (built once); only the camera changes.
pub struct GizmoRenderer {
    pipeline: wgpu::RenderPipeline,
    camera: Camera,
    labels: wgpu::BindGroup,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    index_count: u32,
}

impl GizmoRenderer {
    /// Create the renderer for a surface of the given `format`, with the unit
    /// cube and label atlas uploaded.
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cam-render gizmo shader"),
            source: wgpu::ShaderSource::Wgsl(GIZMO_SHADER.into()),
        });

        let camera = Camera::new(device);
        let (labels, label_layout) = build_label_atlas(device, queue);

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cam-render gizmo pipeline layout"),
            bind_group_layouts: &[&camera.layout, &label_layout],
            push_constant_ranges: &[],
        });

        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<GizmoVertex>() as wgpu::BufferAddress,
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
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: 24,
                    shader_location: 2,
                },
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x2,
                    offset: 36,
                    shader_location: 3,
                },
            ],
        };

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cam-render gizmo pipeline"),
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
                // No culling — depth testing alone resolves the cube's faces,
                // so winding never matters.
                ..Default::default()
            },
            depth_stencil: Some(mesh_depth()),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let (verts, idx) = crate::gizmo::unit_cube();
        let vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cam-render gizmo vertices"),
            contents: bytemuck::cast_slice(&verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let indices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cam-render gizmo indices"),
            contents: bytemuck::cast_slice(&idx),
            usage: wgpu::BufferUsages::INDEX,
        });

        Self {
            pipeline,
            camera,
            labels,
            vertices,
            indices,
            index_count: idx.len() as u32,
        }
    }

    /// Upload the camera view-projection (column-major).
    pub fn set_camera(&self, queue: &wgpu::Queue, view_proj: [[f32; 4]; 4]) {
        self.camera.set(queue, view_proj);
    }

    /// Record the cube draw into an existing render pass.
    pub fn draw(&self, pass: &mut wgpu::RenderPass<'_>) {
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.camera.bind_group, &[]);
        pass.set_bind_group(1, &self.labels, &[]);
        pass.set_vertex_buffer(0, self.vertices.slice(..));
        pass.set_index_buffer(self.indices.slice(..), wgpu::IndexFormat::Uint32);
        pass.draw_indexed(0..self.index_count, 0, 0..1);
    }
}

/// Upload the label atlas to a texture and build its bind group (texture +
/// sampler at group 1), returning both the group and its layout.
fn build_label_atlas(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
) -> (wgpu::BindGroup, wgpu::BindGroupLayout) {
    let (pixels, width, height) = crate::gizmo::label_atlas();
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("cam-render gizmo labels"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &pixels,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(4 * width),
            rows_per_image: Some(height),
        },
        wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
    );
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("cam-render gizmo sampler"),
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        ..Default::default()
    });
    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("cam-render gizmo label layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("cam-render gizmo label bind group"),
        layout: &layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&sampler),
            },
        ],
    });
    (bind_group, layout)
}
