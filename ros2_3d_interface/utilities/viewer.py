import pyglet
from pyglet.window import key
from pyglet.window import mouse
import numpy as np

class Viewer():
    def __init__(self, screen_width, screen_height, cam):
        #Pyglet window initialization
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.cam = cam
        self.window = pyglet.window.Window(
            width=self.screen_width, height=self.screen_height, 
            caption="")
        self.keys = key.KeyStateHandler()
        self.window.push_handlers(self.keys)
        #Mouse initialization
        self.window.push_handlers(self.on_mouse_drag)
        self.window.push_handlers(self.on_mouse_scroll)
    def update(self, dt):
        eye_radius, eye_angle1, eye_angle2 = self.cam.getEyePolar()
        vect = np.array([0.0, 0.0, 0.0])
        #Current camera view direction
        view_x = -np.cos(eye_angle1)
        view_y = -np.sin(eye_angle1)
        #Camera motion
        cam_vel = 5.0
        if self.keys[key.UP] or self.keys[key.W] and not self.keys[key.LSHIFT]:
            vect[0] += cam_vel*dt*view_x
            vect[1] += cam_vel*dt*view_y
        if self.keys[key.DOWN] or self.keys[key.S] and not self.keys[key.LSHIFT]:
            vect[0] -= cam_vel*dt*view_x
            vect[1] -= cam_vel*dt*view_y
        if self.keys[key.LEFT] or self.keys[key.A]:
            vect[0] -= cam_vel*dt*view_y
            vect[1] += cam_vel*dt*view_x
        if self.keys[key.RIGHT] or self.keys[key.D]:
            vect[0] += cam_vel*dt*view_y
            vect[1] -= cam_vel*dt*view_x
        if self.keys[key.LSHIFT] and self.keys[key.W]:
            vect[2] += cam_vel*dt
        if self.keys[key.LSHIFT] and self.keys[key.S]:
            vect[2] -= cam_vel*dt
        self.cam.setEyePos(self.cam.getEyePos()+vect)
        self.cam.setTargetPos(self.cam.getTargetPos()+vect)
        #Quit
        if self.keys[key.ESCAPE]:
            pyglet.app.exit()
    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if buttons & mouse.LEFT:
            eye_radius, eye_angle1, eye_angle2 = self.cam.getEyePolar()
            eye_angle1 -= 0.01*dx
            eye_angle2 += 0.01*dy
            if eye_angle2 > np.pi-0.01:
                eye_angle2 = np.pi-0.01
            if eye_angle2 < 0.01:
                eye_angle2 = 0.01
            self.cam.setEyePolar(eye_radius, eye_angle1, eye_angle2)
    def on_mouse_scroll(self, x, y, dx, dy):
        eye_radius, eye_angle1, eye_angle2 = self.cam.getEyePolar()
        eye_radius -= 1.0*dy
        if eye_radius < 0.02:
            eye_radius = 0.02
        self.cam.setEyePolar(eye_radius, eye_angle1, eye_angle2)

