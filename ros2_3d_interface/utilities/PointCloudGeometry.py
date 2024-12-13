

class ShapePointCloud():
    def __init__(self, ctx, size_points):
        self.ctx = ctx
        #Program shaders
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                uniform mat4 mat_model;
                in vec3 pos;
                in vec3 color;
                out vec3 vertex_color;
                void main() {
                    gl_Position = mat_proj*mat_model*vec4(pos, 1.0);
                    vertex_color = color;
                    gl_PointSize = 3;
                }
            """,
            fragment_shader="""
                #version 330 core
                in vec3 vertex_color;
                out vec4 fragment_color;
                void main() {
                    fragment_color = vec4(vertex_color, 1.0);
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        self.uniform_mat_model = self.prog["mat_model"]
        #Generate vertices
        array_points_rgb = np.ones((size_points,3))
        array_points_xyz = np.zeros((size_points,3))
        #Create buffers
        self.vbo_rgb = ctx.buffer(array_points_rgb.astype(np.float32).tobytes())
        self.vbo_xyz = ctx.buffer(array_points_xyz.astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo_rgb, "3f4", "color"), (self.vbo_xyz, "3f4", "pos")])
    def update_points(self, array_rgb=None, array_xyz=None):
        if array_rgb is not None:
            self.vbo_rgb.write(array_rgb.astype(np.float32).tobytes())
        if array_xyz is not None:
            self.vbo_xyz.write(array_xyz.astype(np.float32).tobytes())
    def render(self, 
            cam, 
            pos=VectZero(), rot=RotIdentity(), 
            scale=1.0):
        self.ctx.line_width = 5.0
        mat_model = MatModel(pos, rot, scale*VectOne())
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.uniform_mat_model.write(mat_model.astype("f4"))
        self.vao.render(mode=moderngl.POINTS)
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()
