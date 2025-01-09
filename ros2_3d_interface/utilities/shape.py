import numpy as np
import moderngl

from ros2_3d_interface.utilities.utils import (
    RotX, RotY, RotZ, RotIdentity, VectZero, VectOne, MatModel, MatNormal
)

class ShapeTrajectory():
    def __init__(self, ctx, points, color=(1.0, 0.0, 0.0, 1.0)):
        self.ctx = ctx
        self.color = color
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                in vec3 pos;
                void main() {
                    gl_Position = mat_proj * vec4(pos, 1.0);
                }
            """,
            fragment_shader="""
                #version 330 core
                uniform vec4 color;
                out vec4 fragment_color;
                void main() {
                    fragment_color = color;
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        self.uniform_color = self.prog["color"]

        self.update_points(points)

    def update_points(self, points):
        if len(points) < 2:
            self.vbo_lines = None
            self.vao_lines = None
            self.vbo_arrows = None
            self.vao_arrows = None
            return

        # Generate line segments
        lines = []
        arrow_triangles = []
        arrow_size = 0.02
        arrow_width = 0.01

        for i in range(len(points) - 1):
            # Line segment
            start = np.array(points[i])
            end = np.array(points[i + 1])
            lines.extend([start, end])

            # Arrowhead (triangle)
            direction = end - start
            direction = direction / np.linalg.norm(direction)

            perp = np.cross(direction, [0, 0, 1])
            if np.linalg.norm(perp) < 1e-6:
                perp = np.cross(direction, [0, 1, 0])
            perp = perp / np.linalg.norm(perp) * arrow_width

            # Define the triangle points
            tip = end
            left = end - direction * arrow_size + perp
            right = end - direction * arrow_size - perp
            arrow_triangles.extend([tip, left, right])

        # Create VBOs and VAOs for lines and arrowheads
        self.vbo_lines = self.ctx.buffer(np.array(lines).astype(np.float32).tobytes())
        self.vao_lines = self.ctx.vertex_array(self.prog, [(self.vbo_lines, "3f4", "pos")])

        self.vbo_arrows = self.ctx.buffer(np.array(arrow_triangles).astype(np.float32).tobytes())
        self.vao_arrows = self.ctx.vertex_array(self.prog, [(self.vbo_arrows, "3f4", "pos")])

    def render(self, cam):
        if self.vao_lines:
            self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
            self.uniform_color.value = self.color
            self.vao_lines.render(mode=moderngl.LINES)

        if self.vao_arrows:
            self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
            self.uniform_color.value = self.color
            self.vao_arrows.render(mode=moderngl.TRIANGLES)

    def release(self):
        self.prog.release()
        if self.vbo_lines:
            self.vbo_lines.release()
        if self.vao_lines:
            self.vao_lines.release()
        if self.vbo_arrows:
            self.vbo_arrows.release()
        if self.vao_arrows:
            self.vao_arrows.release()

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
        self.ctx.line_width = 3.0
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
    def __init__(self, ctx, size_points, point_size=1.0):
        self.ctx = ctx

        self.point_size = point_size

        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                uniform mat4 mat_model;
                uniform mat4 mat_rotation;
                in vec3 pos;
                in vec3 color;
                out vec3 vertex_color;
                void main() {
                    gl_Position = mat_proj * mat_model * mat_rotation * vec4(pos, 1.0);
                    vertex_color = color;
                    gl_PointSize = """ + str(self.point_size) + """;
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
        self.uniform_mat_rotation = self.prog["mat_rotation"]

        array_points_rgb = np.ones((size_points,5))
        array_points_xyz = np.zeros((size_points,5))

        self.vbo_rgb = ctx.buffer(array_points_rgb.astype(np.float32).tobytes())
        self.vbo_xyz = ctx.buffer(array_points_xyz.astype(np.float32).tobytes())

        self.vao = ctx.vertex_array(self.prog, [(self.vbo_rgb, "3f4", "color"), (self.vbo_xyz, "3f4", "pos")])
    def update_points(self, array_rgb=None, array_xyz=None):
        if array_rgb is not None:
            self.vbo_rgb.write(array_rgb.astype(np.float32).tobytes())
        if array_xyz is not None:
            self.vbo_xyz.write(array_xyz.astype(np.float32).tobytes())
    def render(self, cam, pos=VectZero(), rot=RotIdentity(), scale=1.0):
        self.ctx.line_width = 5.0
        mat_model = MatModel(pos, rot, scale * VectOne())
        
        rotation_x = np.array([
            [1,  0,   0,  0],
            [0,  0,  -1,  0],
            [0,  1,   0,  0],
            [0,  0,   0,  1],
        ], dtype=np.float32)
        
        rotation_z = np.array([
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        

        combined_rotation = np.dot(rotation_z, rotation_x)
        
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.uniform_mat_model.write(mat_model.astype("f4"))
        self.uniform_mat_rotation.write(combined_rotation.astype("f4"))
        
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