"""
Rung 4 (rebuilt): "Repeat" — replay a taught trajectory on the follower, now
built on the Arm class instead of raw scservo_sdk.

What changed from the first version:
  - All the port / byteswap / torque / write boilerplate is GONE — it lives in
    servo/arm.py now. This file just loads frames and drives the arm. That's the
    whole point of the Week-2 refactor: write the hardware logic ONCE, reuse it.
  - We use arm.goto() to glide SMOOTHLY to the first recorded pose BEFORE timed
    playback starts. The old version warned "the first frame can jump" and made
    you hand-position the arm near the start; goto() removes that hazard.
  - The body of the trajectory is dense (50 Hz), so we stream those frames with
    write_joints() at the recorded timing — already smooth, no per-frame goto
    (and per-frame goto would fight the recorded dt and re-introduce stutter).

⚠️ SAFETY — read before running:
  - Clear the space around the arm. It WILL move on its own.
  - Keep a hand near the power during the countdown.

Run from the repo ROOT (not from inside teach/):
    conda activate lerobot
    python teach/repeat.py
"""

import os
import sys
import json
import time

# Make `servo` importable when this file is run as `python teach/repeat.py`.
# Running a script puts teach/ (not the repo root) on sys.path, so `import
# servo.arm` would fail without this — we prepend the repo root explicitly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from servo.arm import Arm, FOLLOWER

RECORDING_FILE = "/home/hanson/so-101-arm/data/traj_001.json"

# ---- load the recording (done for you) ----
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

    # ----------------------------------------------------------------------
    # TODO 1 — glide to the START pose, smoothly, before timed playback.
    #   The first recorded pose is  frames[0]["positions"]  — a {name: pos} dict,
    #   exactly the shape goto() wants. Interpolate to it from wherever the arm
    #   is sitting now, so there's no first-frame lurch:
    #       arm.goto(frames[0]["positions"])
    # ----------------------------------------------------------------------
    arm.goto(frames[0]["positions"])
    # ----------------------------------------------------------------------
    # TODO 2 — stream the rest of the trajectory at recorded timing.
    #   The frames are dense, so a plain write_joints() per frame is already
    #   smooth. Walk the frames, command each pose, and sleep the GAP between
    #   consecutive timestamps so playback runs at record speed:
    #       previous_t = frames[0]["t"]
    #       for frame in frames:
    #           arm.write_joints(frame["positions"])
    #           dt = frame["t"] - previous_t
    #           time.sleep(max(0, dt))   # max(0, ...) guards tiny negatives
    #           previous_t = frame["t"]
    # ----------------------------------------------------------------------
    previous_t = frames[0]["t"]
    for frame in frames:
        arm.write_joints(frame["positions"])
        dt = frame["t"] - previous_t
        time.sleep(max(0, dt))
        previous_t = frame["t"]
    print("Replay complete.")
except KeyboardInterrupt:
    print("\nStopped early.")
finally:
    # Leave torque ON so the arm holds its final pose instead of flopping.
    # (Want it limp at the end instead? call arm.torque(False) before close.)
    arm.close()
