import numpy as np

def RotX(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ])

def RotY(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ])

def RotZ(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])

def RotIdentity():
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ])

def VectZero():
    return np.array([0.0, 0.0, 0.0])

def VectOne():
    return np.array([1.0, 1.0, 1.0])

def MatModel(pos=VectZero(), rot=RotIdentity(), scale=VectOne()):
    return np.array([
        [rot[0,0]*scale[0], rot[0,1]*scale[1], rot[0,2]*scale[2], pos[0]],
        [rot[1,0]*scale[0], rot[1,1]*scale[1], rot[1,2]*scale[2], pos[1]],
        [rot[2,0]*scale[0], rot[2,1]*scale[1], rot[2,2]*scale[2], pos[2]],
        [0.0,               0.0,               0.0,               1.0],
    ]).transpose().copy()

def MatNormal(mat_model):
    return np.linalg.inv(mat_model[0:3,0:3]).transpose().copy()

