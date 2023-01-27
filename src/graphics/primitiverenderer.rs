use super::{gl, shader};

#[derive(Clone, Copy, Debug)]
#[repr(u32)]
pub enum PrimitiveType {
    Point = glow::POINTS,
    Line = glow::LINES,
    Filled = glow::TRIANGLES,
}

pub struct PrimitiveRenderer {
    program: shader::Program,
    vertex_array: gl::VertexArray,
    vertex_buffer: gl::VertexBuffer,

    proj_model_view: nalgebra::Matrix4<f32>,
    vertices: Vec<f32>,
    max_vertices: usize,
    vertex_count: usize,
    index: usize,
    active_drawcall: Option<DrawCall>,
    draw_calls: Vec<DrawCall>,
}

#[derive(Clone, Copy, Debug)]
struct DrawCall {
    pt: PrimitiveType,
    start_index: usize,
    vertex_count: usize,
}

impl PrimitiveRenderer {
    pub fn new(gl: &glow::Context, max_vertices: u32) -> Self {
        //load our shader
        let shader = shader::Program::new(
            gl,
            r#"
            layout(location = 0) in vec4 position;
            layout(location = 1) in vec4 color;
            
            uniform mat4 u_projModelView;
            
            varying vec4 v_Color;
            void main(){
                // output the final vertex position
                gl_Position = u_projModelView * position;
                    
                v_Color = vec4(color.xyz, 1.0);
            };
        "#,
            r#"
            layout(location = 0) out vec4 color;

            varying vec4 v_Color;
            void main(){
                color = v_Color;
            };
            "#,
        );

        shader.bind(gl);

        // create the layout description for the program above
        let mut layout = gl::VertexBufferLayout::new();
        layout.push(gl::GLType::Float, 3);
        layout.push(gl::GLType::UnsignedByte, 4);
        let layout = layout;

        let mut vb = gl::VertexBuffer::new(gl, max_vertices * layout.get_stride());

        // allocate storage for our vertices (3 position + 1 color) floats
        let vertices = vec![0f32; max_vertices as usize * 4];

        // create vertex array and combine our vertex buffer with the layout
        let mut va = gl::VertexArray::new(gl);
        va.add_buffer(gl, &mut vb, &layout);

        Self {
            program: shader,
            vertex_array: va,
            vertex_buffer: vb,
            vertices,
            max_vertices: max_vertices as usize,
            proj_model_view: nalgebra::Matrix4::identity(),
            vertex_count: 0,
            index: 0,
            active_drawcall: None,
            draw_calls: Vec::new(),
        }
    }

    pub fn set_mvp(&mut self, mvp: nalgebra::Matrix4<f32>) {
        self.proj_model_view = mvp;
    }

    pub fn vertex(&mut self, x: f32, y: f32, z: f32) {
        assert!(
            self.active_drawcall.is_some(),
            "must call begin() before vertex"
        );

        // if the buffer is full, do a "flush"
        if self.vertex_count >= self.max_vertices - 1 {
            panic!("no more space for vertices");
        }

        self.vertices[self.index + 0] = x;
        self.vertices[self.index + 1] = y;
        self.vertices[self.index + 2] = z;

        self.index += 4; // 3 position + 1 u32 for color
        self.vertex_count += 1;
    }

    pub fn color_rgba(&mut self, r: f32, g: f32, b: f32, a: f32) {
        self.color(Color::rgba(r, g, b, a));
    }

    pub fn color(&mut self, color: Color) {
        self.vertices[self.index + 3] = color.bits;
    }

    pub fn begin(&mut self, primitive_type: PrimitiveType) {
        assert!(
            self.active_drawcall.is_none(),
            "begin cannot be called twice in a row"
        );

        self.active_drawcall = Some(DrawCall {
            pt: primitive_type,
            start_index: self.vertex_count,
            vertex_count: 0,
        });
    }

    pub fn end(&mut self) {
        // mark the current position in the buffer
        if let Some(mut dc) = self.active_drawcall {
            dc.vertex_count = self.vertex_count - dc.start_index;
            self.draw_calls.push(dc);
        } else {
            panic!("end() cannot be called before a call to begin() was made");
        }

        self.active_drawcall = None;
    }

    pub fn draw(&mut self, gl: &glow::Context) {
        use glow::HasContext as _;

        assert!(
            self.active_drawcall.is_none(),
            "end() must be called before draw()"
        );

        // use the shader
        self.program.bind(gl);
        self.program
            .set_uniform_matrix_4_f32(gl, "u_projModelView", self.proj_model_view);

        // upload all our data
        self.vertex_buffer.bind(gl);
        self.vertex_buffer
            .set_vertices(gl, &self.vertices[..self.index]);

        // do the actual drawing using multiple draw calls
        self.vertex_array.bind(gl);

        // TODO: go through and "optimize" the drawcalls if possible, i.e. by combining "adjacent" calls with the same primitive type

        for dc in self.draw_calls.iter() {
            unsafe {
                gl.draw_arrays(dc.pt as u32, dc.start_index as i32, dc.vertex_count as i32);
            }
        }

        // reset state
        self.vertex_count = 0;
        self.index = 0;

        self.draw_calls.clear();
    }

    pub fn destroy(&self, gl: &glow::Context) {
        self.vertex_array.destroy(gl);
        self.vertex_buffer.destroy(gl);
        self.program.destroy(gl);
    }
}

/// An RGBA color.
/// Internally, the color is packed into 4 bytes, one for each of RGBA, instead of as 4 floats to save memory
#[derive(Clone, Copy)]
pub struct Color {
    bits: f32,
}

impl Color {
    pub fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self::rgba(r, g, b, 1.0)
    }
    pub fn rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        let colori = (((255.0 * a) as u32) << 24)
            | (((255.0 * b) as u32) << 16)
            | (((255.0 * g) as u32) << 8)
            | ((255.0 * r) as u32);

        // unsafe { core::mem::transmute::<u32, f32>(colori) }
        Self {
            bits: f32::from_bits(colori),
        }
    }
}
