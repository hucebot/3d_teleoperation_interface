import os
import sys
import glm
import moderngl
import numpy as np
import pygame
import rclpy
from rclpy.node import Node

from ros2_3d_interface.utilities.LineGeometry import LineGeometry
from ros2_3d_interface.utilities.Mesh import Mesh


os.environ['SDL_WINDOWS_DPI_AWARENESS'] = 'permonitorv2'

pygame.init()
pygame.display.set_mode((800, 800), flags=pygame.OPENGL | pygame.DOUBLEBUF, vsync=True)


class PointCloudGeometry:
    def __init__(self, ctx, file_path):
        self.ctx = ctx

        # Load point cloud data from .ply file
        vertices = self.load_ply(file_path)
        self.vertex_count = len(vertices) // 3

        # Create buffer for point cloud
        self.vbo = self.ctx.buffer(vertices.astype('f4').tobytes())

    def load_ply(self, file_path):
        try:
            with open(file_path, 'r') as f:
                lines = f.readlines()

            # Find vertex data start
            header_ended = False
            vertex_data = []
            for line in lines:
                if header_ended:
                    vertex_data.append(list(map(float, line.split()[:3])))
                elif line.startswith("end_header"):
                    header_ended = True

            # Flatten list of vertex positions
            return np.array(vertex_data).flatten()
        except Exception as e:
            print(f"Error loading point cloud: {e}")
            sys.exit(1)

    def vertex_array(self, program):
        return self.ctx.vertex_array(program, [(self.vbo, '3f', 'in_vertex')])


class PointCloudMesh:
    def __init__(self, program, geometry):
        self.ctx = moderngl.get_context()
        self.vao = geometry.vertex_array(program)

    def render(self):
        self.vao.render(moderngl.POINTS)


class Scene:
    def __init__(self):
        self.ctx = moderngl.get_context()

        self.program = self.ctx.program(
            vertex_shader='''
                #version 330 core

                uniform mat4 camera;

                layout (location = 0) in vec3 in_vertex;
                layout (location = 1) in vec3 in_color;

                out vec3 frag_color;

                void main() {
                    frag_color = in_color;
                    gl_Position = camera * vec4(in_vertex, 1.0);
                }
            ''',
            fragment_shader='''
                #version 330 core

                in vec3 frag_color;

                layout (location = 0) out vec4 out_color;

                void main() {
                    out_color = vec4(frag_color, 1.0);
                }
            ''',
        )

        self.line_geometry = LineGeometry()
        self.lines = Mesh(self.program, self.line_geometry)

        # Load point cloud
        pointcloud_path = '/ros2_ws/src/ros2_3d_interface/pointcloud/pointcloud_20241212_164843.ply'
        self.pointcloud_geometry = PointCloudGeometry(self.ctx, pointcloud_path)
        self.pointcloud = PointCloudMesh(self.program, self.pointcloud_geometry)

        # Camera settings
        self.camera_distance = 3.0
        self.camera_yaw = 0.0
        self.camera_pitch = 0.0
        self.mouse_dragging = False  # Flag to track mouse dragging

    def camera_matrix(self):
        # Convert spherical coordinates to Cartesian for camera position
        x = self.camera_distance * glm.cos(glm.radians(self.camera_yaw)) * glm.cos(glm.radians(self.camera_pitch))
        y = self.camera_distance * glm.sin(glm.radians(self.camera_yaw)) * glm.cos(glm.radians(self.camera_pitch))
        z = self.camera_distance * glm.sin(glm.radians(self.camera_pitch))

        eye = (x, y, z)  # Camera position
        center = (0.0, 0.0, 0.0)  # Looking at the origin
        up = (0.0, 0.0, 1.0)  # Up direction
        proj = glm.perspective(glm.radians(45.0), 1.0, 0.1, 1000.0)  # Perspective projection
        look = glm.lookAt(eye, center, up)  # View matrix
        return proj * look

    def render(self):
        camera = self.camera_matrix()

        # Set background color to grey
        self.ctx.clear(0.5, 0.5, 0.5, 1.0)
        self.ctx.enable(self.ctx.DEPTH_TEST)

        self.program['camera'].write(camera)

        # Render axes
        self.lines.render()

        # Render point cloud
        self.pointcloud.render()

    def handle_mouse_motion(self, rel):
        # Update yaw and pitch based on mouse movement
        sensitivity = 0.2
        self.camera_yaw += rel[0] * sensitivity
        self.camera_pitch -= rel[1] * sensitivity
        self.camera_pitch = max(-89.99, min(89.99, self.camera_pitch))  # Limit pitch to avoid gimbal lock

    def handle_mouse_wheel(self, y_offset):
        # Adjust the camera distance (zoom) based on the mouse wheel
        zoom_sensitivity = 0.5
        self.camera_distance -= y_offset * zoom_sensitivity
        self.camera_distance = max(0.0, min(10.0, self.camera_distance))  # Limit zoom range

    def start_drag(self):
        # Reset mouse relative movement at the start of dragging
        pygame.mouse.get_rel()  # Reset any accumulated movement
        self.mouse_dragging = True

    def stop_drag(self):
        self.mouse_dragging = False


class SceneNode(Node):
    def __init__(self):
        super().__init__('scene_node')
        self.scene = Scene()
        self.timer = self.create_timer(0.016, self.render_callback)  # ~60 FPS
        self.get_logger().info("SceneNode initialized and running.")

    def render_callback(self):
        # Handle Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.destroy_node()
                rclpy.shutdown()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    self.scene.start_drag()  # Reset mouse movement and start dragging
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:  # Left mouse button
                    self.scene.stop_drag()  # Stop dragging
            elif event.type == pygame.MOUSEMOTION:
                if self.scene.mouse_dragging:  # Only rotate if mouse is dragging
                    rel = pygame.mouse.get_rel()
                    self.scene.handle_mouse_motion(rel)
            elif event.type == pygame.MOUSEWHEEL:
                self.scene.handle_mouse_wheel(event.y)  # Use y offset for zoom

        self.scene.render()
        pygame.display.flip()


def main(args=None):
    rclpy.init(args=args)
    node = SceneNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
