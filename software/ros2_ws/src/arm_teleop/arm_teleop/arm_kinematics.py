#!/usr/bin/env python3
"""
Pure-numpy forward kinematics + geometric Jacobian for a serial revolute arm
parsed from a URDF string. No KDL / MoveIt dependency.

Unlike the legacy `damped_servo.py` kinematics (which hard-coded rotation about
+Z and ignored each joint's <axis>), this version reads the real joint axis and
rotates about it via Rodrigues' formula. That matters for the dicerox arm, whose
Joint4 and Joint6 have axis "0 0 -1" — hard-coding +Z would silently flip their
sign and corrupt the Jacobian.
"""

from __future__ import annotations

import numpy as np
from urdf_parser_py.urdf import URDF


def _rpy_to_rot(r: float, p: float, y: float) -> np.ndarray:
    """Roll-Pitch-Yaw (URDF fixed-axis XYZ) to a 3x3 rotation matrix."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _origin_to_T(xyz, rpy) -> np.ndarray:
    """URDF <origin> (xyz, rpy) to a 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_rot(axis: np.ndarray, angle: float) -> np.ndarray:
    """4x4 rotation of `angle` about a unit `axis` (Rodrigues' formula)."""
    x, y, z = axis
    K = np.array([[0.0, -z, y],
                  [z, 0.0, -x],
                  [-y, x, 0.0]])
    R = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    return T


class ArmKinematics:
    """FK + geometric Jacobian for the actuated revolute chain base -> tip."""

    def __init__(self, urdf_string: str, base: str = 'base_link', tip: str = 'Link6'):
        robot = URDF.from_xml_string(urdf_string)
        by_child = {j.child: j for j in robot.joints}

        # Trace the kinematic path from tip back to base, then reverse it.
        path, current = [], tip
        while current != base:
            if current not in by_child:
                raise ValueError(
                    f'Cannot reach base "{base}" from tip "{tip}": '
                    f'link "{current}" has no parent joint.')
            joint = by_child[current]
            path.append(joint)
            current = joint.parent
        path.reverse()

        self.origins = []        # fixed 4x4 transform before each joint
        self.axes = []           # local unit axis for each joint (or None if fixed)
        self.is_actuated = []    # bool per joint in the path
        self.joint_names = []    # name per actuated joint, in order
        lower, upper = [], []

        for j in path:
            xyz = list(j.origin.xyz) if (j.origin and j.origin.xyz) else [0.0, 0.0, 0.0]
            rpy = list(j.origin.rpy) if (j.origin and j.origin.rpy) else [0.0, 0.0, 0.0]
            self.origins.append(_origin_to_T(xyz, rpy))

            actuated = j.type in ('revolute', 'continuous')
            self.is_actuated.append(actuated)
            if actuated:
                axis = np.array(j.axis if j.axis else [0.0, 0.0, 1.0], dtype=float)
                n = np.linalg.norm(axis)
                self.axes.append(axis / n if n > 0 else np.array([0.0, 0.0, 1.0]))
                self.joint_names.append(j.name)
                if j.type == 'continuous' or j.limit is None:
                    lower.append(-np.pi)
                    upper.append(np.pi)
                else:
                    lower.append(j.limit.lower)
                    upper.append(j.limit.upper)
            else:
                self.axes.append(None)

        self.lower = np.array(lower)
        self.upper = np.array(upper)
        self.n_act = int(sum(self.is_actuated))

    def fk(self, q: np.ndarray):
        """Forward kinematics.

        Returns (T_ee, axes_world, p_joints) where axes_world[i] and p_joints[i]
        are the world-frame rotation axis and origin of actuated joint i, taken
        *before* that joint's own rotation (as needed by the geometric Jacobian).
        """
        T = np.eye(4)
        axes_world, p_joints = [], []
        qi = 0
        for k, origin in enumerate(self.origins):
            T = T @ origin
            if self.is_actuated[k]:
                axes_world.append(T[:3, :3] @ self.axes[k])
                p_joints.append(T[:3, 3].copy())
                T = T @ _axis_rot(self.axes[k], q[qi])
                qi += 1
        return T, axes_world, p_joints

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """6 x n_act geometric Jacobian in the base frame.

        Rows 0-2: linear velocity of the tip. Rows 3-5: angular velocity.
        """
        T_ee, axes_world, p_joints = self.fk(q)
        p_ee = T_ee[:3, 3]
        J = np.zeros((6, self.n_act))
        for i in range(self.n_act):
            J[:3, i] = np.cross(axes_world[i], p_ee - p_joints[i])
            J[3:, i] = axes_world[i]
        return J
