"""
kinematics/fk.py — forward kinematics for the SO-101.

A homogeneous transform is a 4x4 matrix packing a 3x3 rotation (top-left) and a
3x1 translation (top-right column) with a [0,0,0,1] bottom row. It answers "where
is this frame, and how is it oriented, as seen from its parent?" — so a whole
kinematic chain is just a product of these matrices.

Joint geometry comes from kinematics/urdf_params.py.

    python kinematics/fk.py      # runs the self-checks
"""

import os
import sys
import numpy as np

# Let `python kinematics/fk.py` import the sibling module by absolute path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kinematics.urdf_params import JOINTS as URDF_JOINTS


def translate(x, y, z):
    """4x4 homogeneous transform: pure translation, no rotation."""
    return np.array([
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_z(theta):
    """4x4 homogeneous rotation of `theta` radians about +Z (yaw)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_x(theta):
    """4x4 homogeneous rotation about +X (roll)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0,   c,  -s, 0.0],
        [0.0,   s,   c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_y(theta):
    """4x4 homogeneous rotation about +Y (pitch).

    Note the sign pattern is transposed relative to rot_x/rot_z: +s top-right,
    -s bottom-left.
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [  c, 0.0,   s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [ -s, 0.0,   c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_rpy(roll, pitch, yaw):
    """4x4 transform for a URDF roll-pitch-yaw orientation.

    URDF applies roll about X, then pitch about Y, then yaw about Z, all about
    FIXED axes — so pre-multiply, rightmost applied first.
    """
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


# Positioning joints only — the gripper joint works the jaw, it doesn't move
# where the hand is, so the chain stops at wrist_roll.
CHAIN = [j for j in URDF_JOINTS if j[0] != "gripper"]


def pose_of(angles):
    """Joint angles (radians) -> 4x4 pose of the gripper frame in the base frame.

    `angles` is a dict {joint_name: radians}, as returned by Arm.read_joints_rad().
    A joint missing from the dict is treated as 0.0.

    Each joint contributes translate(xyz) @ rot_rpy(rpy) @ rot_z(theta): walk out
    to where the joint sits, tilt into its mounting orientation, then apply the
    servo's rotation. Transforms accumulate on the left, so each link's offset is
    carried through every rotation before it.
    """
    T = np.eye(4)
    for name, xyz, rpy, axis, limits in CHAIN:
        theta = angles.get(name, 0.0)
        joint_T = translate(*xyz) @ rot_rpy(*rpy) @ rot_z(theta)
        T = T @ joint_T
    return T


if __name__ == "__main__":
    # Self-checks — run `python kinematics/fk.py`. All must pass.
    import numpy as np

    # 1) rot_z(0) is the identity (rotating by nothing changes nothing).
    assert np.allclose(rot_z(0.0), np.eye(4)), "rot_z(0) should be identity"

    # 2) rot_z(pi/2) sends the x-axis point (1,0,0) to (0,1,0).
    p = np.array([1.0, 0.0, 0.0, 1.0])         # homogeneous point on +X
    out = rot_z(np.pi / 2) @ p
    assert np.allclose(out, [0.0, 1.0, 0.0, 1.0]), f"got {out}, want [0,1,0,1]"
    print("rot_z passes both checks ✅")

    # 3) A single non-zero angle must reduce to that one axis' rotation.
    assert np.allclose(rot_rpy(0, 0, 0), np.eye(4)), "rpy(0,0,0) should be identity"
    assert np.allclose(rot_rpy(0.3, 0, 0), rot_x(0.3)), "pure roll should equal rot_x"
    assert np.allclose(rot_rpy(0, 0.3, 0), rot_y(0.3)), "pure pitch should equal rot_y"
    assert np.allclose(rot_rpy(0, 0, 0.3), rot_z(0.3)), "pure yaw should equal rot_z"

    # 4) ORDER check (this is the one that catches a wrong multiply order):
    #    roll=pitch=pi/2 sends the x-axis point (1,0,0) to (0,0,-1).
    out = rot_rpy(np.pi / 2, np.pi / 2, 0) @ np.array([1.0, 0.0, 0.0, 1.0])
    assert np.allclose(out, [0.0, 0.0, -1.0, 1.0]), f"rpy order wrong: got {out}, want [0,0,-1,1]"
    print("rot_rpy passes all checks ✅")

    # 5) pose_of at the URDF home (all joint angles = 0). The exact position needs
    #    the ruler check, but the output must be a valid homogeneous transform and
    #    the reach must be physically plausible. The lower bound matters: a chain
    #    that fails to accumulate returns the identity, whose reach is 0.
    T = pose_of({})                    # empty dict -> every joint 0.0
    assert np.allclose(T[3, :], [0, 0, 0, 1]), "bottom row must be [0,0,0,1]"
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3)), "rotation block must be orthonormal"
    assert np.isclose(np.linalg.det(R), 1.0), "rotation det must be +1"
    reach = np.linalg.norm(T[:3, 3])
    assert 0.1 < reach < 0.5, f"reach {reach:.3f} m is not physically plausible — chain is wrong"
    print(f"pose_of valid ✅  home position (m): {np.round(T[:3, 3], 4)}  reach {reach:.3f}")
