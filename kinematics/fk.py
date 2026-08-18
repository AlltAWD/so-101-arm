"""
kinematics/fk.py — forward kinematics for the SO-101.

Built from three homogeneous-transform primitives (translate, rot_z, rot_rpy),
chained down the joints in kinematics/urdf_params.py. A homogeneous transform is
a 4x4 matrix that packs a 3x3 rotation (top-left) and a 3x1 translation (top-right
column) together, with a [0,0,0,1] bottom row, so a whole kinematic chain is just
a product of these matrices.

We build up in pieces:
    translate(x,y,z)  -- DONE, your template
    rot_z(theta)      -- YOU write this now
    rot_rpy(r,p,y)    -- next
    pose_of(angles)   -- next, the chain itself
"""

import os
import sys
import numpy as np

# Let `python kinematics/fk.py` import the sibling module by absolute path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kinematics.urdf_params import JOINTS as URDF_JOINTS


def translate(x, y, z):
    """4x4 homogeneous transform: pure translation, no rotation.

    The rotation block is the 3x3 identity (nothing spins); the translation lives
    in the top-right column. This is your template for the shape of a 4x4.
    """
    return np.array([
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_z(theta):
    """4x4 homogeneous transform: rotation of `theta` radians about the +Z axis.

    No translation, so the top-right column is zeros. Fill the top-left 3x3 block
    with the z-rotation:
        x' = x*cos - y*sin
        y' = x*sin + y*cos
        z' = z            (z is untouched -> its row/column stay identity)

    Watch which corner the minus sign goes in.
    """
    c, s = np.cos(theta), np.sin(theta)
    # TODO(you): return the 4x4 numpy array. Use the translate() shape above as a
    # guide, but put c/s into the top-left 3x3 and leave the translation column 0.
    return np.array([
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_x(theta):
    """4x4 homogeneous rotation about +X (roll). DONE — study the sign pattern.
    X is untouched, so its row/column stay identity; c/s mix y and z."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0,   c,  -s, 0.0],
        [0.0,   s,   c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_y(theta):
    """4x4 homogeneous rotation about +Y (pitch). DONE — NOTE THE SIGN FLIP.
    Y is untouched; c/s mix z and x. Unlike rot_x/rot_z the +s is top-right and
    the -s is bottom-left (the pattern is 'transposed'). This is a classic gotcha."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [  c, 0.0,   s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [ -s, 0.0,   c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_rpy(roll, pitch, yaw):
    """4x4 homogeneous transform for a URDF roll-pitch-yaw orientation.

    URDF applies roll about X first, then pitch about Y, then yaw about Z, all
    about the FIXED axes -> pre-multiply, so:
        R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    (rightmost = applied first = roll). Compose the three you already have.
    """
    # TODO(you): return the product of the three rotation matrices, in the order
    # above. Matrix multiply is `@` in numpy.
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


# Positioning joints only — the gripper joint just works the jaw, it doesn't
# move where the hand is, so we stop the chain at wrist_roll.
CHAIN = [j for j in URDF_JOINTS if j[0] != "gripper"]


def pose_of(angles):
    """Forward kinematics: joint angles (radians) -> 4x4 pose of the gripper
    frame in the base frame.

    `angles` is a dict {joint_name: radians} — exactly what Arm.read_joints_rad()
    returns. A joint missing from the dict is treated as 0.0.

    Walk base -> wrist. Start T at the identity, and for each joint fold its
    transform onto the running product:
        joint_T = translate(*xyz) @ rot_rpy(*rpy) @ rot_z(theta)
        T = T @ joint_T
    Each tuple in CHAIN is (name, xyz, rpy, axis, limits); you need name, xyz, rpy.
    """
    T = np.eye(4)
    for name, xyz, rpy, axis, limits in CHAIN:
        theta = angles.get(name, 0.0)
        # TODO(you): build joint_T from translate/rot_rpy/rot_z (order above),
        #   then accumulate it onto T. Two lines.
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

    # 5) pose_of at the URDF home (all joint angles = 0). We can't know the exact
    #    position yet (that's the ruler step), but the output MUST be a valid
    #    homogeneous transform, and the reach must be physically plausible.
    T = pose_of({})                    # empty dict -> every joint 0.0
    assert np.allclose(T[3, :], [0, 0, 0, 1]), "bottom row must be [0,0,0,1]"
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3)), "rotation block must be orthonormal"
    assert np.isclose(np.linalg.det(R), 1.0), "rotation det must be +1"
    reach = np.linalg.norm(T[:3, 3])
    assert reach < 0.5, f"reach {reach:.3f} m exceeds the arm's length — chain is wrong"
    print(f"pose_of valid ✅  home position (m): {np.round(T[:3, 3], 4)}  reach {reach:.3f}")
