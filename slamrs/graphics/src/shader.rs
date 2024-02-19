use eframe::glow;

/// Builds upon `glow::Program` to easily construct a new shader program in a safe way
pub struct Program {
    program: glow::Program,
}

impl Program {
    /// Create a new shader program from the vertex and fragment shader source
    /// Panics if the shader could not be compiled.
    pub fn new(
        gl: &glow::Context,
        vertex_shader_source: &str,
        fragment_shader_source: &str,
    ) -> Self {
        use glow::HasContext as _;

        let shader_version = if cfg!(target_arch = "wasm32") {
            "#version 300 es"
        } else {
            "#version 330"
        };

        unsafe {
            let program = gl.create_program().expect("Cannot create program");

            let shader_sources = [
                (glow::VERTEX_SHADER, vertex_shader_source),
                (glow::FRAGMENT_SHADER, fragment_shader_source),
            ];

            let shaders: Vec<_> = shader_sources
                .iter()
                .map(|(shader_type, shader_source)| {
                    let source = format!("{}\n{}", shader_version, shader_source);
                    let shader = gl
                        .create_shader(*shader_type)
                        .expect("Cannot create shader");
                    gl.shader_source(shader, &source);
                    gl.compile_shader(shader);
                    assert!(
                        gl.get_shader_compile_status(shader),
                        "Failed to compile shader of type {shader_type}: {}, source: {}",
                        gl.get_shader_info_log(shader),
                        &source
                    );
                    gl.attach_shader(program, shader);
                    shader
                })
                .collect();

            gl.link_program(program);
            if !gl.get_program_link_status(program) {
                panic!("{}", gl.get_program_info_log(program));
            }

            for shader in shaders {
                gl.detach_shader(program, shader);
                gl.delete_shader(shader);
            }

            Self { program }
        }
    }

    pub fn set_uniform_matrix_4_f32(
        &self,
        gl: &glow::Context,
        name: &str,
        value: nalgebra::Matrix4<f32>,
    ) {
        use glow::HasContext as _;
        unsafe {
            gl.uniform_matrix_4_f32_slice(
                gl.get_uniform_location(self.program, name).as_ref(),
                false,
                value.as_slice(),
            );
        }
    }

    pub fn destroy(&self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe {
            gl.delete_program(self.program);
        }
    }

    pub fn bind(&self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe {
            gl.use_program(Some(self.program));
        }
    }

    pub fn unbind(&self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe {
            gl.use_program(None);
        }
    }
}
