import numpy as np
import moderngl

from ros2_3d_interface.utilities.utils import (
    RotX, RotY, RotZ, RotIdentity, VectZero, VectOne, MatModel, MatNormal
)

class ShapeTrajectory():
    def __init__(self, ctx, points):
        self.ctx = ctx
        # Program shaders
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                in vec3 pos;
                void main() {
                    gl_Position = mat_proj * vec4(pos, 1.0);
                    gl_PointSize = 20.0;
                }
            """,
            fragment_shader="""
                #version 330 core
                out vec4 fragment_color;
                void main() {
                    fragment_color = vec4(1.0, 0.0, 0.0, 1.0);
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]

        self.vbo = ctx.buffer(np.array(points).astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f4", "pos")])

    def render(self, cam):
        self.ctx.point_size = 10.0
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.vao.render(mode=moderngl.POINTS)

    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()


class ShapeGrid():
    def __init__(self, ctx, length=10.0, segments=10):
        self.ctx = ctx
        #Program shaders
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                in vec3 pos;
                void main() {
                    gl_Position = mat_proj*vec4(pos, 1.0);
                }
            """,
            fragment_shader="""
                #version 330 core
                out vec4 fragment_color;
                void main() {
                    fragment_color = vec4(1.0, 1.0, 1.0, 1.0);
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        #Generate vertices
        list_vertex = []
        for x in np.linspace(-length, length, 2*segments+1):
            list_vertex += [x, length, 0.0]
            list_vertex += [x, -length, 0.0]
        for y in np.linspace(-length, length, 2*segments+1):
            list_vertex += [length, y, 0.0]
            list_vertex += [-length, y, 0.0]
        #Create buffers
        self.vbo = ctx.buffer(np.array(list_vertex).astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f4", "pos")])
    def render(self, cam):
        self.ctx.line_width = 1.0
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.vao.render(mode=moderngl.LINES);
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()

class ShapeFrame():
    def __init__(self, ctx):
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
        list_data = []
        list_data += [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        list_data += [1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        list_data += [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        list_data += [0.0, 1.0, 0.0, 0.0, 1.0, 0.0]
        list_data += [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        list_data += [0.0, 0.0, 1.0, 0.0, 0.0, 1.0]
        #Create buffers
        self.vbo = ctx.buffer(np.array(list_data).astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f4 3f4", "pos", "color")])
    def render(self, cam, pos=VectZero(), rot=RotIdentity(), scale=1.0):
        self.ctx.line_width = 5.0
        mat_model = MatModel(pos, rot, scale*VectOne())
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.uniform_mat_model.write(mat_model.astype("f4"))
        self.vao.render(mode=moderngl.LINES);
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()

class ShapePyramid():
    def __init__(self, ctx):
        self.ctx = ctx
        #Program shaders
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                uniform mat4 mat_model;
                in vec3 pos;
                void main() {
                    gl_Position = mat_proj*mat_model*vec4(pos, 1.0);
                }
            """,
            fragment_shader="""
                #version 330 core
                uniform vec4 obj_color;
                out vec4 fragment_color;
                void main() {
                    fragment_color = obj_color;
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        self.uniform_mat_model = self.prog["mat_model"]
        self.uniform_obj_color = self.prog["obj_color"]
        #Generate vertices
        list_data = []
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, 0.0, 0.0]
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, 1.0, 1.0]
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, -1.0, 1.0]
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, 1.0, -1.0]
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, -1.0, -1.0]
        list_data += [1.0, 1.0, 1.0]
        list_data += [1.0, -1.0, 1.0]
        list_data += [1.0, -1.0, 1.0]
        list_data += [1.0, -1.0, -1.0]
        list_data += [1.0, -1.0, -1.0]
        list_data += [1.0, 1.0, -1.0]
        list_data += [1.0, 1.0, -1.0]
        list_data += [1.0, 1.0, 1.0]
        near = 0.1
        list_data += [near, near, near]
        list_data += [near, -near, near]
        list_data += [near, -near, near]
        list_data += [near, -near, -near]
        list_data += [near, -near, -near]
        list_data += [near, near, -near]
        list_data += [near, near, -near]
        list_data += [near, near, near]
        #Create buffers
        self.vbo = ctx.buffer(np.array(list_data).astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f4", "pos")])
    def render(self, 
            cam, pos=VectZero(), rot=RotIdentity(), 
            fovx=60.0, aspect=1.0, far=20.0,
            color=[1.0, 1.0, 1.0, 1.0]):
        self.ctx.line_width = 3.0
        length_y = far*np.tan(fovx*np.pi/180.0/2.0)
        lepgth_z = length_y/aspect
        mat_model = MatModel(pos, rot, [far, length_y, lepgth_z])
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.uniform_mat_model.write(mat_model.astype("f4"))
        self.uniform_obj_color.write(np.array(color).astype("f4"))
        self.vao.render(mode=moderngl.LINES);
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()

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

class ShapeQuadTexture():
    def __init__(self, ctx, tex_width, tex_height):
        #Program shaders
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                uniform mat4 mat_model;
                in vec3 pos;
                in vec2 uv;
                out vec2 vertex_uv;
                void main() {
                    gl_Position = mat_proj*mat_model*vec4(pos, 1.0);
                    vertex_uv = uv;
                }
            """,
            fragment_shader="""
                #version 330 core
                uniform sampler2D texture;
                in vec2 vertex_uv;
                out vec4 fragment_color;
                void main() {
                    fragment_color = vec4(texture2D(texture, vertex_uv).rgb, 1.0);
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        self.uniform_mat_model = self.prog["mat_model"]
        #Generate vertices
        list_data = []
        list_data += [0.0, -1.0, -1.0] + [1.0,1.0]
        list_data += [0.0, 1.0, -1.0] + [0.0, 1.0]
        list_data += [0.0, 1.0, 1.0] + [0.0, 0.0]
        list_data += [0.0, -1.0, -1.0] + [1.0, 1.0]
        list_data += [0.0, 1.0, 1.0] + [0.0, 0.0]
        list_data += [0.0, -1.0, 1.0] + [1.0, 0.0]
        #Create buffers
        self.vbo = ctx.buffer(np.array(list_data).astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f4 2f4", "pos", "uv")])
        #Create texture
        self.texture = ctx.texture((tex_width,tex_height), 3, dtype="f4")
    def update_texture(self, array_pixels):
        self.texture.write(array_pixels.astype(np.float32).tobytes())
    def render(self, 
            cam, pos=VectZero(), rot=RotIdentity(), 
            size=[1.0, 1.0]):
        self.texture.use()
        mat_model = MatModel(pos, rot, [1.0, size[0]/2, size[1]/2])
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.uniform_mat_model.write(mat_model.astype("f4"))
        self.vao.render(mode=moderngl.TRIANGLES);
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()

