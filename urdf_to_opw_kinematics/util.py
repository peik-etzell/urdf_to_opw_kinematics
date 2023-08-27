#!/usr/bin/env python
import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation
from urdf_parser_py.urdf import Joint

TOL = 1e-6


def angle(v1, v2):
    cos = np.dot(v1, v2) / (norm(v1) * norm(v2))
    sin = norm(np.cross(v1, v2))
    return np.arctan2(sin, cos)


def rot_y(alpha):
    return np.array(
        [
            [np.cos(alpha), 0, np.sin(alpha)],
            [0, 1, 0],
            [-np.sin(alpha), 0, np.cos(alpha)]
        ]
    )


class Transform:
    def __init__(self, xyz: np.array, rpy: np.array):
        rotation = Rotation.from_euler('xyz', rpy).as_matrix()
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = xyz
        self.transform = transform

    def __mul__(self, other):
        return np.dot(self.transform, other.transform)

    def rotation(self):
        return self.transform[:3, :3]


def transform(joint: Joint) -> np.array:
    translation = np.array(joint.origin.position)
    rotation = Rotation.from_euler('xyz', joint.origin.rotation).as_matrix()
    isometry = np.eye(4)
    isometry[:3, :3] = rotation
    isometry[:3, 3] = translation
    return isometry


class Axis:
    # def __init__(self, position, position_rel, direction):
    def __init__(self, transform: np.array, axis: np.array):
        # self.position = position
        # self.p_rel = position_rel
        # self.direction = direction / norm(direction)
        self.transform = transform
        self.position = transform[:3, 3]
        self.direction = np.round(np.dot(transform[:3, :3], axis))

    def __str__(self):
        s = "[Axis]\txyz: {:20} axis: {:10}".format(
            str(np.round(self.position, 4)), str(self.direction))

        # s += str(self.position) + "\tdir: "
        # s += str(self.direction) + "\n"
        return s

    def is_perpendicular(self, other):
        c = norm(np.dot(self.direction, other.direction))
        return (c < TOL)

    def is_parallel(self, other):
        c = norm(np.cross(self.direction, other.direction))
        return (c < TOL)


def _distance_vector(axes1, axes2):
    if axes1.is_parallel(axes2):
        v = axes2.position - axes1.position
        return v - np.dot(v, axes1.direction) * axes1.direction
    else:
        v = axes2.position - axes1.position
        d = np.cross(axes1.direction, axes2.direction)
        return np.dot(v, d) / norm(d) * d


def distance(axes1, axes2, return_vector=False, along=None):
    v = _distance_vector(axes1, axes2)
    if (along is not None):
        return np.abs(np.dot(v, along.direction))
    if (not return_vector):
        return norm(v)
    return v
