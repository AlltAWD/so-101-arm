"""servo/hold_test.py — command the zero pose, measure where each joint settles.

Run from the repo ROOT:  python servo/hold_test.py
"""
import os
import sys
import time

# (1) same sys.path line as teach/repeat.py — so `import servo.arm` works
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# (2) from servo.arm import Arm, FOLLOWER
from servo.arm import Arm, FOLLOWER

ARM_JOINTS = ["shoulder_pan", 
              "shoulder_lift", 
              "elbow_flex", 
              "wrist_flex", 
              "wrist_roll"]

# (3) Arm(FOLLOWER), open(), load_calibration()
arm = Arm(FOLLOWER)
arm.open()
arm.load_calibration()
# (4) try:
#         torque(True)
#         build targets: {name: <0 rad converted to ticks> for each ARM_JOINT}
#         goto(targets)
#         sleep ~2 s to settle
#         read_joints_rad() 20 times, 0.05 s apart, collecting each joint's readings
#         print each joint's average
#     finally:
#         input("Hand under the arm, then Enter...")
#         torque(False); close()
try:
    arm.torque(True)
    targets = {name: arm.rad_to_ticks(name, 0) for name in ARM_JOINTS}
    arm.goto(targets)
    time.sleep(2)
    samples = {name: [] for name in ARM_JOINTS}
    for i in range(20):
        reading = arm.read_joints_rad()
        for name in ARM_JOINTS:
            if reading[name] is not None:
                samples[name].append(reading[name])
        time.sleep(0.05)
    for name in ARM_JOINTS:
        xs = samples[name]
        avg = sum(xs) / len(xs)
        print(f"{name:14s} {avg:+.4f} rad  ({len(xs)} reads)")
finally:
    input("Torque will be disabled, make sure to protect arm, then press Enter...")
    arm.torque(False)
    arm.close()
