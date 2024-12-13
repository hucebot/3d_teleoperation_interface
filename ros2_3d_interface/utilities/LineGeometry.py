import moderngl
import numpy as np

class LineGeometry:
    def __init__(self):
        self.ctx = moderngl.get_context()
        # Define vertices for X, Y, Z axes (lines from origin in typical colors)
        vertices = np.array([
            # X-axis (red)
            0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
            10.0, 0.0, 0.0, 10.0, 0.0, 0.0,
            # Y-axis (green)
            0.0, 0.0, 0.0, 0.0, 10.0, 0.0,
            0.0, 10.0, 0.0, 0.0, 10.0, 0.0,
            # Z-axis (blue)
            0.0, 0.0, 0.0, 0.0, 0.0, 10.0,
            0.0, 0.0, 10.0, 0.0, 0.0, 10.0,
        ])
        self.vbo = self.ctx.buffer(vertices.astype('f4').tobytes())

    def vertex_array(self, program):
        return self.ctx.vertex_array(program, [(self.vbo, '3f 3f', 'in_vertex', 'in_color')])