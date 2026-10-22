"""
kinematics/cross_check_tf.py: compare pose_of() with tf2_echo base_link gripper_link.

    python kinematics/cross_check_tf.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kinematics.fk import pose_of

# TODO 1: the same pose you're publishing on /joint_states in T2.
#         pose_of takes a dict {joint_name: radians}; the gripper isn't needed.
pose = {'shoulder_pan': 0.5, 
        'shoulder_lift': -0.4, 
        'elbow_flex': 0.8, 
        'wrist_flex': 0.3, 
        'wrist_roll': 0.2, 
        'gripper': 0.0}
# TODO 2: T = pose_of(...)
T = pose_of(pose)
# TODO 3: print the position T[:3, 3] and the rotation T[:3, :3], rounded to 4 decimals
#         (np.round, or np.set_printoptions(precision=4, suppress=True))
print("Translation: ", np.round(T[:3, 3], 4))
print("Rotation: ", np.round(T[:3, :3], 4))