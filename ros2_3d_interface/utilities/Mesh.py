import moderngl

class Mesh:
    def __init__(self, program, geometry):
        self.ctx = moderngl.get_context()
        self.vao = geometry.vertex_array(program)

    def render(self):
        self.vao.render(moderngl.LINES)