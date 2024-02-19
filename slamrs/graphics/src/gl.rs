use eframe::glow;

pub struct VertexBufferLayout {
    elements: Vec<LayoutElement>,
    stride: u32,
}

#[derive(Clone, Copy)]
struct LayoutElement {
    gl_type: GLType,
    count: u32,
    normalized: bool,
}

#[derive(Clone, Copy)]
#[repr(u32)]
pub enum GLType {
    Float = glow::FLOAT,
    UnsignedInt = glow::UNSIGNED_INT,
    UnsignedByte = glow::UNSIGNED_BYTE,
}

impl GLType {
    fn size(&self) -> u32 {
        match *self {
            GLType::Float => 4,
            GLType::UnsignedInt => 4,
            GLType::UnsignedByte => 1,
        }
    }
}

impl VertexBufferLayout {
    pub fn new() -> Self {
        Self {
            elements: Vec::new(),
            stride: 0,
        }
    }

    pub fn push(&mut self, gl_type: GLType, count: u32) -> &mut Self {
        assert!((1..=4).contains(&count), "count must be 1,2,3 or 4");
        self.elements.push(LayoutElement {
            gl_type,
            count,
            normalized: match gl_type {
                GLType::Float => false,
                GLType::UnsignedInt => false,
                GLType::UnsignedByte => true,
            },
        });
        self.stride += (gl_type.size()) * count;

        self
    }

    pub fn get_stride(&self) -> u32 {
        self.stride
    }
}

pub struct VertexArray {
    va: glow::VertexArray,
}

impl VertexArray {
    pub fn new(gl: &glow::Context) -> Self {
        use glow::HasContext as _;

        let vertex_array = unsafe {
            gl.create_vertex_array()
                .expect("Cannot create vertex array")
        };

        Self { va: vertex_array }
    }
    /// Adds a buffer to this VertexArray together with the specified layout
    pub fn add_buffer(
        &mut self,
        gl: &glow::Context,
        buffer: &mut VertexBuffer,
        layout: &VertexBufferLayout,
    ) {
        use glow::HasContext as _;
        self.bind(gl);
        buffer.bind(gl);

        // attach the layout (this binds the layout and VBO to the VAO)
        let mut offset = 0u32;

        for (i, e) in layout.elements.iter().enumerate() {
            unsafe {
                gl.enable_vertex_attrib_array(i as u32);
                gl.vertex_attrib_pointer_f32(
                    i as u32,
                    e.count as i32,
                    e.gl_type as u32,
                    e.normalized,
                    layout.get_stride() as i32,
                    offset as i32,
                );

                offset += e.count * e.gl_type.size();
            }
        }
    }

    pub fn bind(&self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe {
            gl.bind_vertex_array(Some(self.va));
        }
    }

    pub fn unbind(&self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe {
            gl.bind_vertex_array(None);
        }
    }

    pub fn destroy(&self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe { gl.delete_vertex_array(self.va) }
    }
}

pub struct VertexBuffer {
    id: glow::Buffer,
    is_bound: bool,
}
impl VertexBuffer {
    pub fn new(gl: &glow::Context) -> Self {
        use glow::HasContext as _;

        let buffer = unsafe { gl.create_buffer().expect("Cannot create vertex buffer") };

        Self {
            id: buffer,
            is_bound: false,
        }
    }

    pub fn set_vertices(&mut self, gl: &glow::Context, vertices: &[f32]) {
        use glow::HasContext as _;

        if !self.is_bound {
            self.bind(gl);
        }

        // reinterpret the data as pure bytes
        let data = unsafe {
            std::slice::from_raw_parts(
                vertices.as_ptr() as *const u8,
                std::mem::size_of_val(vertices),
            )
        };
        // upload the data
        unsafe { gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, data, glow::DYNAMIC_DRAW) }
    }

    pub fn bind(&mut self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe {
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.id));
        }
        self.is_bound = true;
    }

    pub fn unbind(&mut self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe {
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
        }
        self.is_bound = false;
    }

    pub fn destroy(&self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe { gl.delete_buffer(self.id) }
    }
}
