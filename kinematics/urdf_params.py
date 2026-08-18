"""
kinematics/urdf_params.py — SO-101 kinematic constants, verbatim from the CAD.

Source: TheRobotStudio/SO-ARM100, Simulation/SO101/so101_new_calib.urdf
(generated from the Onshape CAD by onshape-to-robot). These are the DESIGN
dimensions — more accurate than hand measurement.

Each joint's transform from its parent link is:
    Trans(origin_xyz) @ Rot_rpy(origin_rpy) @ Rot_z(theta)
where theta is the joint angle and every joint spins about its own local +Z.

CONVENTION WARNING: this file is the *new_calib* URDF — joint zero = MIDDLE of
each joint's range. Our data/calib.json defined zero at the fully-EXTENDED pose
(that's the *old_calib* convention). So URDF theta and our calibrated theta differ
by a per-joint constant we still have to reconcile when we build FK. Do not assume
they're equal.

Units: meters, radians. Order: base -> gripper.
"""

import numpy as np

# name: (origin_xyz, origin_rpy, axis, (lower, upper))
JOINTS = [
    ("shoulder_pan",  (0.0388353, -8.97657e-09,  0.0624),   (3.14159,  4.18253e-17, -3.14159), (0, 0, 1), (-1.91986, 1.91986)),
    ("shoulder_lift", (-0.0303992, -0.0182778,  -0.0542),   (-1.5708, -1.5708,       0.0),     (0, 0, 1), (-1.74533, 1.74533)),
    ("elbow_flex",    (-0.11257,   -0.028,       1.73763e-16), (0.0,     0.0,         1.5708),  (0, 0, 1), (-1.69,    1.69)),
    ("wrist_flex",    (-0.1349,     0.0052,      3.62355e-17), (0.0,     0.0,        -1.5708),  (0, 0, 1), (-1.65806, 1.65806)),
    ("wrist_roll",    (5.55112e-17, -0.0611,     0.0181),    (1.5708,  0.0486795,   3.14159),  (0, 0, 1), (-2.74385, 2.84121)),
    ("gripper",       (0.0202,      0.0188,     -0.0234),    (1.5708, -5.24284e-08, -1.41553e-15), (0, 0, 1), (-0.174533, 1.74533)),
]
