"""
servo/calibrate.py — capture the tick->radian ZERO for each joint.

Why this exists
---------------
The STS3215 reports an ABSOLUTE encoder count: 0-4095 ticks over one full
revolution (4096 ticks = 2*pi rad). But its zero is arbitrary — wherever the
horn happened to sit at power-on. Forward kinematics needs joint ANGLES measured
from a pose we define. So we pick a reference, read the ticks there, and store
those as per-joint OFFSETS. After that:

    rad = sign * (ticks - offset) * (2*pi / 4096)

The scale is known exactly (4096 ticks/rev), so we only need the offset.

The convention: zero = middle of the range
------------------------------------------
`kinematics/urdf_params.py` is the *new_calib* URDF, whose zero is the MIDDLE OF
EACH JOINT'S RANGE — not a straight or extended pose. If calibration defines zero
anywhere else, `ticks_to_rad()` and `pose_of()` measure from different origins and
FK is wrong by a constant per-joint error. An earlier hand-posed "straight home"
reference was off by ~12 deg on shoulder_pan and elbow_flex for exactly this
reason.

So we match the URDF by definition rather than by careful posing:

    offset = (ticks at one mechanical stop + ticks at the other) / 2

You can't see "the middle of the range," but you can feel both ends of it. The
hardware defines the pose, so it is reproducible.

Capturing at two stops beats sampling a continuous sweep: a running min/max loop
needs threading to watch for a keypress while reading, and a joint that crosses
the 0/4095 encoder boundary silently records min=0, max=4095. Capturing
deliberately at two known stops has no wrap problem — you know which end you
are at.

How to run
----------
    conda activate lerobot
    python servo/calibrate.py                # calibrates the FOLLOWER

Per joint: torque goes off, you move the joint to one stop and press Enter, then
to the opposite stop and press Enter. The measured range is checked against the
URDF's declared range so a bad capture is caught while you are still at the arm.
"""

import sys
import os
import json
import time

# Let `python servo/calibrate.py` find the `servo` package (script dir, not repo
# root, is what lands on sys.path — same bootstrap teach/repeat.py needed).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from servo.arm import Arm, FOLLOWER, JOINTS, RAD_PER_TICK
from kinematics.urdf_params import JOINTS as URDF_JOINTS

CALIB_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "calib.json")

REFERENCE = ("zero = midpoint of each joint's mechanical range, matching the "
             "new_calib URDF convention in kinematics/urdf_params.py")

# {name: (lower, upper)} radians — the URDF's ground truth for how wide each
# joint's travel should be, used to sanity-check what we measure.
URDF_LIMITS = {name: limits for name, xyz, rpy, axis, limits in URDF_JOINTS}

# Joints with no usable hard stops: wrist_roll spins freely, so "both ends of the
# range" is meaningless for it. Skipping it is safe because wrist_roll is the last
# link in the FK chain and its rot_z is a pure rotation — its angle changes the
# gripper's orientation without moving the fingertip position at all.
NO_HARD_STOPS = {"wrist_roll"}

SPAN_WARN = 0.10      # warn if measured range differs from URDF by more than this
SAMPLES = 10          # reads to average at each stop
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


def capture(arm, name, which):
    """Hold `name` at one stop and return that joint's averaged tick reading.

    Reads every joint (average_reads is all-or-nothing) but keeps only the one
    we asked about.
    """
    input(f"  Move '{name}' to its {which} stop, hold steady, press Enter... ")
    readings, good = average_reads(arm)
    ticks = readings[name]
    print(f"    captured {ticks} ticks (averaged over {good} reads)")
    return ticks


def sweep_joint(arm, name):
    """Capture both mechanical stops of one joint. Returns (low, high) in ticks.

    Which physical end gives the smaller tick value isn't known until it's read,
    so capture in whatever order the arm is swung and sort afterwards.
    """
    first = capture(arm, name, "first")
    second = capture(arm, name, "opposite")
    return (min(first, second), max(first, second))


def midpoint_offset(low, high):
    """The tick value halfway between the two stops — our new zero.

    Rounded, not truncated: `//` would shave up to a tick off every joint in the
    same direction, biasing the whole calibration.
    """
    return round((low + high) / 2)


def check_span(name, low, high):
    """Compare measured travel against the URDF's declared travel.

    Two independent sources for the same quantity — a physical sweep and the CAD
    file — so agreement means the capture is trustworthy. On this arm the good
    joints agree to ~1%; anything past SPAN_WARN means a missed stop or an
    encoder wrap.

    Returns the fractional error (positive = measured wider than URDF).
    """
    measured_span = (high - low) * RAD_PER_TICK
    urdf_span = URDF_LIMITS[name][1] - URDF_LIMITS[name][0]
    error = (measured_span - urdf_span) / urdf_span
    print(f"    span: measured {measured_span:.3f} rad  "
          f"urdf {urdf_span:.3f} rad  err {error:+.1%}")
    return error


def main():
    arm = Arm(FOLLOWER)
    arm.open()
    try:
        arm.torque(False)   # go limp so you can hand-pose it
        print("Torque OFF — the arm is back-drivable.")
        print("\nWe'll do one joint at a time: move it to one stop, then the other.")
        print("Be gentle at the stops — find them, don't force them.\n")

        # Keep the previous offsets so joints we skip don't lose their value.
        old = {}
        if os.path.exists(CALIB_PATH):
            with open(CALIB_PATH) as f:
                old = json.load(f).get("joints", {})

        joints_out = {}
        for name in JOINTS:
            if name in NO_HARD_STOPS:
                kept = old.get(name, {}).get("offset")
                print(f"[{name}] no hard stops — skipping sweep, keeping offset {kept}")
                joints_out[name] = {"offset": kept, "sign": 1, "no_hard_stops": True}
                continue

            print(f"[{name}]")
            low, high = sweep_joint(arm, name)
            offset = midpoint_offset(low, high)
            error = check_span(name, low, high)
            if abs(error) > SPAN_WARN:
                print(f"    ⚠  measured range is off by {error:+.1%} — suspect capture, "
                      f"consider redoing this joint")
            print(f"    offset = {offset}\n")

            joints_out[name] = {
                "offset": offset, "sign": 1,
                "range_min": low, "range_max": high,
                "span_error": round(error, 4),
            }

        calib = {
            "reference_pose": REFERENCE,
            "ticks_per_rev": 4096,
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "joints": joints_out,
        }

        os.makedirs(os.path.dirname(CALIB_PATH), exist_ok=True)
        with open(CALIB_PATH, "w") as f:
            json.dump(calib, f, indent=2)
        print(f"Saved -> {CALIB_PATH}")
        print("\nVerify: pose the arm anywhere, then compare")
        print("    pose_of(arm.read_joints_rad())  vs  a ruler.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
