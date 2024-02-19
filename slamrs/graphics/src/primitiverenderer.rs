use super::{gl, shader};
use eframe::glow;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

/* /// Test for using a "RenderGuard" to make sure state of the renderer is correctly managed
pub struct RenderGuard<'a> {
    pr: &'a mut PrimitiveRenderer,
    pt: PrimitiveType,
    start_index: usize,
}

impl RenderGuard<'_> {
    pub fn end(self) {
        // the end of this method will drop self

        // self.pr.draw_calls.push(DrawCall {
        //     pt: self.pt,
        //     start_index: self.start_index,
        //     vertex_count: self.pr.vertex_count - self.start_index,
        // });

        // // TODO: remove
        // self.pr.active_drawcall = None;
    }
}

impl Vertex3C for RenderGuard<'_> {
    fn xyzc(&mut self, x: f32, y: f32, z: f32, color: Color) {
        self.pr.xyzc(x, y, z, color);
    }
}

impl Drop for RenderGuard<'_> {
    fn drop(&mut self) {
        self.pr.draw_calls.push(DrawCall {
            pt: self.pt,
            start_index: self.start_index,
            vertex_count: self.pr.vertex_count - self.start_index,
        });

        // TODO: remove
        self.pr.active_drawcall = None;
    }
}
 */

pub trait Vertex3C {
    /// Adds a vertex at a 3D position with a specific color
    fn xyzc(&mut self, x: f32, y: f32, z: f32, color: Color);

    #[inline]
    fn xyz(&mut self, x: f32, y: f32, z: f32) {
        self.xyzc(x, y, z, Color::BLACK);
    }

    #[inline]
    fn v3(&mut self, v: nalgebra::Vector3<f32>) {
        self.v3c(v, Color::BLACK);
    }
    #[inline]
    fn v3c(&mut self, v: nalgebra::Vector3<f32>, color: Color) {
        self.xyzc(v.x, v.y, v.z, color);
    }
}

pub trait Vertex2C {
    /// Adds a vertex at a 2D position with a specific color
    fn xyc(&mut self, x: f32, y: f32, color: Color);

    #[inline]
    fn xy(&mut self, x: f32, y: f32) {
        self.xyc(x, y, Color::BLACK);
    }

    #[inline]
    fn v2(&mut self, v: nalgebra::Vector2<f32>) {
        self.v2c(v, Color::BLACK);
    }
    #[inline]
    fn v2c(&mut self, v: nalgebra::Vector2<f32>, color: Color) {
        self.xyc(v.x, v.y, color);
    }
}

/// Automatically implement Vertex2C for any Vertex3C by setting z=0.0
impl<T: Vertex3C> Vertex2C for T {
    fn xyc(&mut self, x: f32, y: f32, color: Color) {
        self.xyzc(x, y, 0.0, color);
    }
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
            
            out vec4 v_Color;
            void main(){
                // output the final vertex position
                gl_Position = u_projModelView * position;
                    
                v_Color = vec4(color.xyz, 1.0);
            }
        "#,
            r#"
            precision mediump float;
            layout(location = 0) out vec4 color;
    
            in vec4 v_Color;
            void main(){
                color = v_Color;
            }
            "#,
        );

        shader.bind(gl);

        // create the layout description for the program above
        let mut layout = gl::VertexBufferLayout::new();
        layout.push(gl::GLType::Float, 3);
        layout.push(gl::GLType::UnsignedByte, 4);
        let layout = layout;

        let mut vb = gl::VertexBuffer::new(gl);

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

    /*
    pub fn begin2(&mut self, primitive_type: PrimitiveType) -> RenderGuard<'_> {
        // todo: remove me later
        self.active_drawcall = Some(DrawCall {
            pt: primitive_type,
            start_index: self.vertex_count,
            vertex_count: 0,
        });

        RenderGuard {
            start_index: self.vertex_count,
            pr: self,
            pt: primitive_type,
        }
    }*/

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

    // TODO: add function for ensuring space for X more vertices. That could actually take in the GL context and perform a `draw` if necessary...

    pub fn flush(&mut self, gl: &glow::Context) {
        use glow::HasContext as _;

        assert!(
            self.active_drawcall.is_none(),
            "end() must be called before draw()"
        );

        // println!(
        //     "Flushing {} vertices in {} draw calls => {:.2} vertices / call. Cap = {} ~= {} MB",
        //     self.vertex_count,
        //     self.draw_calls.len(),
        //     self.vertex_count as f32 / self.draw_calls.len() as f32,
        //     self.vertices.capacity(),
        //     (self.vertices.capacity() * std::mem::size_of::<f32>()) / 1024 / 1024
        // );
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

impl Vertex3C for PrimitiveRenderer {
    fn xyzc(&mut self, x: f32, y: f32, z: f32, color: Color) {
        assert!(
            self.active_drawcall.is_some(),
            "must call begin() before vertex"
        );

        // if the buffer is full, do a "flush"
        if self.vertex_count >= self.max_vertices - 1 {
            panic!("no more space for vertices");
        }

        // SAFETY: we keep track and make sure we have enough space using index and vertex_count variables
        #[allow(clippy::identity_op)]
        unsafe {
            *self.vertices.get_unchecked_mut(self.index + 0) = x;
            *self.vertices.get_unchecked_mut(self.index + 1) = y;
            *self.vertices.get_unchecked_mut(self.index + 2) = z;
            *self.vertices.get_unchecked_mut(self.index + 3) = color.bits;
        }
        // self.vertices[self.index + 0] = x;
        // self.vertices[self.index + 1] = y;
        // self.vertices[self.index + 2] = z;
        // self.vertices[self.index + 3] = color.bits;

        self.index += 4; // 3 position + 1 u32 for color
        self.vertex_count += 1;
    }
}

/// An RGBA color.
/// Internally, the color is packed into 4 bytes, one for each of RGBA, instead of as 4 floats to save memory
#[derive(Clone, Copy)]
pub struct Color {
    bits: f32,
}

impl Color {
    pub const BLACK: Color = Color::rgba_u8(0x00, 0x00, 0x00, 0xff);
    pub const WHITE: Color = Color::rgba_u8(0xff, 0xff, 0xff, 0xff);
    pub const RED: Color = Color::rgba_u8(0xff, 0x00, 0x00, 0xff);
    pub const GREEN: Color = Color::rgba_u8(0x00, 0xff, 0x00, 0xff);
    pub const BLUE: Color = Color::rgba_u8(0x00, 0x00, 0xff, 0xff);

    pub fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self::rgba(r, g, b, 1.0)
    }
    pub fn rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self::rgba_u8(
            (255.0 * r) as u8,
            (255.0 * g) as u8,
            (255.0 * b) as u8,
            (255.0 * a) as u8,
        )
    }

    pub const fn rgba_u8(r: u8, g: u8, b: u8, a: u8) -> Self {
        let colori = ((a as u32) << 24) | ((b as u32) << 16) | ((g as u32) << 8) | (r as u32);

        // bits: f32::from_bits(colori), // (not const yet...)
        Self {
            bits: unsafe { core::mem::transmute::<u32, f32>(colori) },
        }
    }

    pub fn grayscale(gray: f32) -> Self {
        Self::rgb(gray, gray, gray)
    }
}

impl From<[f32; 3]> for Color {
    fn from(value: [f32; 3]) -> Self {
        Color::rgb(value[0], value[1], value[2])
    }
}

impl From<[f32; 4]> for Color {
    fn from(value: [f32; 4]) -> Self {
        Color::rgba(value[0], value[1], value[2], value[3])
    }
}
