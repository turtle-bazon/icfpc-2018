use gfx;
use gfx::handle::{DepthStencilView, RenderTargetView};
use gfx::traits::FactoryExt;
use gfx::PipelineState;
use std::collections::hash_map::{HashMap, Entry};

#[derive(Debug)]
pub enum Error {
    CreateBuffer(gfx::buffer::CreationError),
    ResizeBuffer(gfx::buffer::CreationError),
    UpdateBuffer(gfx::UpdateError<usize>),
    CreatePipelineState(gfx::PipelineStateError<String>),
    CreateShaderSet(gfx::shade::ProgramError),
}

pub struct VoxelRenderer<R, F> where R: gfx::Resources, F: gfx::Factory<R> {
    vertex_data: Vec<Vertex>,
    vertex_buffer: gfx::handle::Buffer<R, Vertex>,
    pso_map: HashMap<gfx::format::Format, PipelineState<R, pipe::Meta>>,
    shaders: gfx::ShaderSet<R>,
    factory: F,
}

impl<R, F> VoxelRenderer<R, F> where R: gfx::Resources, F: gfx::Factory<R> {
    pub fn new(mut factory: F, initial_buffer_size: usize) -> Result<VoxelRenderer<R, F>, Error> {
        let set = factory.create_shader_set(VERTEX_SRC, FRAGMENT_SRC)
            .map_err(Error::CreateShaderSet)?;
        let vertex_buffer = factory.create_buffer(
            initial_buffer_size,
            gfx::buffer::Role::Vertex,
            gfx::memory::Usage::Dynamic,
            gfx::memory::Bind::empty()
        ).map_err(Error::CreateBuffer)?;

        Ok(VoxelRenderer {
            vertex_data: Vec::new(),
            vertex_buffer: vertex_buffer,
            pso_map: HashMap::new(),
            shaders: set,
            factory,
        })
    }

    fn prepare_pso(&mut self, format: gfx::format::Format) -> Result<(), Error> where {
        Ok(if let Entry::Vacant(e) = self.pso_map.entry(format) {
            let init = pipe::Init {
                vbuf: (),
                u_model_view_proj: "u_model_view_proj",
                out_color: ("o_Color", format, gfx::state::ColorMask::all(), Some(gfx::preset::blend::ALPHA)),
                out_depth: gfx::preset::depth::LESS_EQUAL_WRITE,
            };
            let pso = self.factory.create_pipeline_state(
                &self.shaders,
                gfx::Primitive::TriangleList,
                gfx::state::Rasterizer::new_fill(),
                init,
            ).map_err(Error::CreatePipelineState)?;
            e.insert(pso);
        })
    }

    pub fn draw_voxel(&mut self, min: [f32; 3], max: [f32; 3], color: [f32; 4]) {
        // front
        self.vertex_data.push(Vertex{position: [min[0], min[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], min[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], max[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], min[1], min[2]], color: color});
        // back
        self.vertex_data.push(Vertex{position: [min[0], min[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], min[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], min[1], max[2]], color: color});
        // left
        self.vertex_data.push(Vertex{position: [min[0], min[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], min[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], max[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], min[1], min[2]], color: color});
        // right
        self.vertex_data.push(Vertex{position: [max[0], min[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], min[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], min[1], min[2]], color: color});
        // top
        self.vertex_data.push(Vertex{position: [min[0], max[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], max[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], max[1], min[2]], color: color});
        // bottom
        self.vertex_data.push(Vertex{position: [min[0], min[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], min[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], min[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], min[1], max[2]], color: color});
        self.vertex_data.push(Vertex{position: [max[0], min[1], min[2]], color: color});
        self.vertex_data.push(Vertex{position: [min[0], min[1], min[2]], color: color});
    }

    pub fn render<C, T> (
        &mut self,
        encoder: &mut gfx::Encoder<R, C>,
        color_target: &RenderTargetView<R, T>,
        depth_target: &DepthStencilView<R, gfx::format::DepthStencil>,
        projection: [[f32; 4]; 4],
    )
        -> Result<(), Error> where
        C: gfx::CommandBuffer<R>,
        T: gfx::format::RenderFormat,
    {
        use gfx::memory::{Typed, Usage, Bind};

        if self.vertex_data.len() > self.vertex_buffer.len() {
            let mut size = self.vertex_buffer.len();
            while size < self.vertex_data.len() {
                size *= 2;
            }
            self.vertex_buffer = self.factory.create_buffer(size, gfx::buffer::Role::Vertex, Usage::Dynamic, Bind::empty())
                .map_err(Error::ResizeBuffer)?;
        }

        encoder.update_buffer(&self.vertex_buffer, &self.vertex_data[..], 0)
            .map_err(Error::UpdateBuffer)?;

        self.prepare_pso(T::get_format())?;
        let pso = &self.pso_map[&T::get_format()];

        let data = pipe::Data {
            vbuf: self.vertex_buffer.clone(),
            u_model_view_proj: projection,
            out_color: color_target.raw().clone(),
            out_depth: depth_target.clone(),
        };

        let slice = gfx::Slice::new_match_vertex_buffer(&self.vertex_buffer);
        encoder.draw(&slice, pso, &data);

        self.vertex_data.clear();
        Ok(())
    }
}

static VERTEX_SRC: &'static [u8] = b"
    #version 150 core

    uniform mat4 u_model_view_proj;
    in vec3 at_position;
    in vec4 at_color;
    out vec4 v_color;

    void main() {
        gl_Position = u_model_view_proj * vec4(at_position, 1.0);
        v_color = at_color;
    }
";

static FRAGMENT_SRC: &'static [u8]= b"
    #version 150

    in vec4 v_color;
    out vec4 out_color;

    void main() {
        out_color = v_color;
    }
";

gfx_vertex_struct!( Vertex {
    position: [f32; 3] = "at_position",
    color: [f32; 4] = "at_color",
});

gfx_pipeline_base!( pipe {
    vbuf: gfx::VertexBuffer<Vertex>,
    u_model_view_proj: gfx::Global<[[f32; 4]; 4]>,
    out_color: gfx::RawRenderTarget,
    out_depth: gfx::DepthTarget<::gfx::format::DepthStencil>,
});
