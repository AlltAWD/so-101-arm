"""
Rung 4: "Repeat" — replay a taught trajectory on the follower, built on the Arm class.

goto() glides to the first recorded pose before timed playback starts, which
removes the first-frame lurch. The body of the trajectory is already dense (50 Hz),
so those frames stream through write_joints() at the recorded timing — a per-frame
goto() would fight the recorded dt and reintroduce stutter.

⚠️ Clear the space around the arm; it will move on its own. Keep a hand near the power.

Run from the repo ROOT (not from inside teach/):
    conda activate lerobot
    python teach/repeat.py
"""

import os
import sys
import json
import time

# Running a script puts teach/ (not the repo root) on sys.path, so `import
# servo.arm` needs the repo root prepended explicitly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from servo.arm import Arm, FOLLOWER

RECORDING_FILE = "/home/hanson/so-101-arm/data/traj_001.json"

with open(RECORDING_FILE, "r") as f:
    frames = json.load(f)

if not frames:
    print("No frames loaded. Run teach/teach.py first to record a trajectory.")
    raise SystemExit

arm = Arm(FOLLOWER)
arm.open()
try:
    arm.torque(True)   # motors now hold + obey position commands

    print(f"Loaded {len(frames)} frames. Hands clear — about to move.")
    for n in (3, 2, 1):
        print(f"  starting in {n}...")
        time.sleep(1)

    arm.goto(frames[0]["positions"])

    # Sleep the gap between consecutive timestamps so playback runs at record speed.
    previous_t = frames[0]["t"]
    for frame in frames:
        arm.write_joints(frame["positions"])
        dt = frame["t"] - previous_t
        time.sleep(max(0, dt))   # max(0, ...) guards tiny negatives
        previous_t = frame["t"]
    print("Replay complete.")
except KeyboardInterrupt:
    print("\nStopped early.")
finally:
    # Leave torque ON so the arm holds its final pose instead of flopping.
    # (Want it limp at the end? call arm.torque(False) before close.)
    arm.close()
