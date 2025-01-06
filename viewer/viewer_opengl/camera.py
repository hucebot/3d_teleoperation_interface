import numpy as np
from pyrr import Matrix44
from viewer_opengl.utils import RotIdentity, RotZ, VectZero

class Camera():
    def __init__(self):
        self.eye_x = 1.0
        self.eye_y = 0.0
        self.eye_z = 1.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.fovx = 60.0
        self.viewport = (0, 0, 0, 0)
    def setParams(self, fovx, viewport):
        self.fovx = fovx
        self.viewport = viewport
    def getFovx(self):
        return self.fovx
    def getFovy(self):
        return 2.0*np.arctan(np.tan(self.fovx*np.pi/180.0/2.0)/self.getAspect())*180.0/np.pi
    def getAspect(self):
        return self.viewport[2]/self.viewport[3]
    def getViewport(self):
        return self.viewport
    def getEyePos(self):
        return np.array([self.eye_x, self.eye_y, self.eye_z])
    def getEyeRot(self):
        vectX = self.getTargetPos() - self.getEyePos()
        vectY = np.cross(np.array([0.0, 0.0, 1.0]), vectX)
        vectZ = np.cross(vectX, vectY)
        return np.array([
            vectX/np.linalg.norm(vectX),
            vectY/np.linalg.norm(vectY),
            vectZ/np.linalg.norm(vectZ)
        ]).transpose()
    def getTargetPos(self):
        return np.array([self.target_x, self.target_y, self.target_z])
    def setEyePos(self, vect):
        self.eye_x = vect[0]
        self.eye_y = vect[1]
        self.eye_z = vect[2]
    def setTargetPos(self, vect):
        self.target_x = vect[0]
        self.target_y = vect[1]
        self.target_z = vect[2]
    def getEyePolar(self):
        vect = self.getEyePos() - self.getTargetPos()
        eye_radius = np.linalg.norm(vect)
        eye_angle1 = np.arctan2(vect[1], vect[0])
        eye_angle2 = np.arccos(vect[2]/eye_radius)
        return eye_radius, eye_angle1, eye_angle2
    def setEyePolar(self, eye_radius, eye_angle1, eye_angle2):
        self.eye_x = self.target_x + eye_radius*np.cos(eye_angle1)*np.sin(eye_angle2)
        self.eye_y = self.target_y + eye_radius*np.sin(eye_angle1)*np.sin(eye_angle2)
        self.eye_z = self.target_z + eye_radius*np.cos(eye_angle2)
    def getMatProj(self):
        mat_proj = Matrix44.perspective_projection(
            fovy=self.getFovy(), 
            aspect=self.getAspect(),
            near=0.01, 
            far=1000.0)
        mat_view = Matrix44.look_at(
            eye=self.getEyePos(),
            target=self.getTargetPos(),
            up=[0.0, 0.0, 1.0])
        return mat_proj * mat_view
    def getProjPlanePos(self, dist):
        return self.getEyePos()+dist*self.getEyeRot()@np.array([1.0,0.0,0.0])
    def getProjPlaneRot(self):
        return self.getEyeRot()@RotZ(np.pi)
    def getProjPlaneSize(self, dist):
        return [
            2.0*dist*np.tan(self.getFovx()*np.pi/180.0/2.0), 
            2.0*dist*np.tan(self.getFovx()*np.pi/180.0/2.0)/self.getAspect()]

class CameraOffAxis():
    def __init__(self):
        self.eye_x = 1.0
        self.eye_y = 0.0
        self.eye_z = 1.0
        self.proj_center = VectZero()
        self.proj_rot = RotIdentity()
        self.proj_half_width = 1.0
        self.proj_half_height = 1.0
        self.viewport = (0, 0, 0, 0)
    def setParams(self, viewport):
        self.viewport = viewport
    def setProjection(self, center, rot, half_width, half_height):
        self.proj_center = center
        self.proj_rot = rot
        self.proj_half_width = half_width
        self.proj_half_height = half_height
    def getViewport(self):
        return self.viewport
    def getEyePos(self):
        return np.array([self.eye_x, self.eye_y, self.eye_z])
    def setEyePos(self, vect):
        self.eye_x = vect[0]
        self.eye_y = vect[1]
        self.eye_z = vect[2]
    def getMatProj(self):
        vect = self.proj_rot@(self.getEyePos() - self.proj_center)
        mat_proj = Matrix44.perspective_projection_bounds(
            left=-vect[1]-self.proj_half_width,
            right=-vect[1]+self.proj_half_width,
            bottom=-vect[2]+self.proj_half_height,
            top=-vect[2]-self.proj_half_height,
            near=vect[0],
            far=1000.0)
        mat_view = Matrix44.look_at(
            eye=self.getEyePos(),
            target=self.getEyePos()-self.proj_rot@np.array([1.0,0.0,0.0]),
            up=[0.0, 0.0, 1.0])
        return mat_proj * mat_view

