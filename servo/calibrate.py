"""
servo/calibrate.py — capture the tick->radian ZERO for each joint.

Why this exists
---------------
The STS3215 reports an ABSOLUTE encoder count: 0-4095 ticks over one full
revolution (4096 ticks = 2*pi rad). But its zero is arbitrary — wherever the
horn happened to sit at power-on. Forward kinematics needs joint ANGLES measured
from a pose we define. So we pick a reference pose, read the ticks there, and
store those as per-joint OFFSETS. After that:

    rad = sign * (ticks - offset) * (2*pi / 4096)

The scale is known exactly (4096 ticks/rev), so we only need the offset. `sign`
defaults to +1; flip it later for any joint that FK shows running backwards.

How to run
----------
    conda activate lerobot
    python servo/calibrate.py                # calibrates the FOLLOWER

Steps it walks you through:
  1. Torque goes OFF so the arm is back-drivable.
  2. You hand-pose the arm to the REFERENCE pose and hold it steady.
  3. Press Enter — it averages a few reads (noise reduction) and saves
     data/calib.json.

Reference pose (recommended): the SO-101 "straight home" — links aligned as in
the standard URDF, gripper closed. Reproduce it the same way each time.
"""

import sys
import os
import json
import time

# Let `python servo/calibrate.py` find the `servo` package (script dir, not repo
# root, is what lands on sys.path — same bootstrap teach/repeat.py needed).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from servo.arm import Arm, FOLLOWER, JOINTS

CALIB_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "calib.json")

# Describe the pose you posed the arm into, so future-you knows how to redo it.
REFERENCE_POSE = "SO-101 straight home: links aligned per standard URDF, gripper closed"

SAMPLES = 10          # reads to average at the reference pose
SAMPLE_DT = 0.02      # seconds between samples


def average_reads(arm, n=SAMPLES, dt=SAMPLE_DT):
    """Read all joints n times and average per joint — smooths encoder jitter.

    Builds a running sum per joint, then divides. If any read comes back None
    (comm hiccup), we skip that whole sample rather than corrupt the average.
    """
    sums = {name: 0 for name in JOINTS}
    good = 0
    for _ in range(n):
        pos = arm.read_joints()
        if any(v is None for v in pos.values()):
            continue
        for name in JOINTS:
            sums[name] += pos[name]
        good += 1
        time.sleep(dt)
    if good == 0:
        raise RuntimeError("No good reads — check the arm connection.")
    return {name: round(sums[name] / good) for name in JOINTS}, good


def main():
    arm = Arm(FOLLOWER)
    arm.open()
    try:
        arm.torque(False)   # go limp so you can hand-pose it
        print("Torque OFF — the arm is back-drivable.")
        print(f"\nPose the arm to:\n    {REFERENCE_POSE}")
        input("Hold it steady, then press Enter to capture... ")

        offsets, good = average_reads(arm)
        print(f"Captured (averaged over {good} reads): {offsets}")

        calib = {
            "reference_pose": REFERENCE_POSE,
            "ticks_per_rev": 4096,
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "joints": {
                name: {"offset": offsets[name], "sign": 1} for name in JOINTS
            },
        }

        os.makedirs(os.path.dirname(CALIB_PATH), exist_ok=True)
        with open(CALIB_PATH, "w") as f:
            json.dump(calib, f, indent=2)
        print(f"\nSaved -> {CALIB_PATH}")
        print("At this pose every joint now reads ~0 rad. Verify with:")
        print("    python servo/arm.py   (once read_joints_rad is wired in)")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
