"""
kinematics/validate_fk.py — check FK predictions against ruler measurements.

Every check in fk.py tests the math against itself. This one tests it against the
world, which is the only way to catch a wrong joint zero.

Runs in two phases so you never have to hold the arm and a ruler at once:

  1. RECORD  — torque off, you hand-pose the arm, press Enter. Repeat for a few
               poses. Only the tick positions are stored.
  2. MEASURE — torque on, the arm drives itself back to each recorded pose and
               HOLDS there. Hands free: measure, type the numbers in.

What to measure: pose_of() stops the chain at wrist_roll, so the predicted point
is the CENTRE OF THE WRIST ROLL AXIS — not the fingertips, which are ~3.6 cm
further out.

Frame: origin is the centre of the baseplate at table level. The shoulder_pan
axis sits 6.24 cm above the base plane and 3.88 cm forward of x=0, so find the
pan axis, drop to the table, come back 3.88 cm. +x is the direction the arm
reaches, +z is up.

Tip: keep shoulder_pan near zero and the arm stays in the x-z plane, so y should
read ~0 and you only have two numbers to measure per pose.

Run from the repo ROOT:
    conda activate lerobot
    python kinematics/validate_fk.py
"""

import os
import sys
import json
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from servo.arm import Arm, FOLLOWER
from kinematics.fk import pose_of

OUT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "fk_validation.json")

TARGET_CM = 1.0     # agreement we're aiming for
PAN_WARN = 0.15     # rad; past this the arm is out of the x-z plane and y matters


def ask_float(prompt):
    """Read a number from the user. Blank skips the pose (returns None)."""
    while True:
        raw = input(prompt).strip()
        if raw == "":
            return None
        try:
            return float(raw)
        except ValueError:
            print("    (enter a number, or blank to skip this pose)")


def record_poses(arm):
    """Torque off; capture tick positions for as many poses as the user wants."""
    arm.torque(False)
    print("\n--- PHASE 1: record poses ---")
    print("Torque OFF. Pose the arm by hand, press Enter to capture.")
    print("Aim for 3-4 poses spread out: folded, extended, high, low.")
    print("Keep shoulder_pan near its zero so the arm stays in the x-z plane.")
    print("Press Enter on an empty prompt when you're done.\n")

    poses = []
    while True:
        raw = input(f"  Pose {len(poses) + 1}: hold it, press Enter (or 'q' to finish)... ")
        if raw.strip().lower() == "q":
            break
        ticks = arm.read_joints()
        if any(v is None for v in ticks.values()):
            print("    a servo read failed — try that pose again")
            continue
        poses.append(ticks)
        print(f"    captured {ticks}")
        if len(poses) >= 3:
            print("    (3+ poses recorded — 'q' to move on to measuring)")
    return poses


def measure_pose(arm, ticks, index):
    """Drive to one recorded pose, print the prediction, collect the measurement."""
    print(f"\n[pose {index}] moving...")
    arm.goto(ticks)
    time.sleep(0.5)

    rad = arm.read_joints_rad()
    if any(v is None for v in rad.values()):
        print("    a servo read failed — skipping this pose")
        return None

    T = pose_of(rad)
    px, py, pz = T[:3, 3]

    if abs(rad["shoulder_pan"]) > PAN_WARN:
        print(f"    note: shoulder_pan is {np.degrees(rad['shoulder_pan']):.0f} deg off zero, "
              f"so the arm is out of the x-z plane (predicted y = {py * 100:.1f} cm)")

    print(f"    predicted:  x = {px * 100:6.1f} cm   y = {py * 100:6.1f} cm   z = {pz * 100:6.1f} cm")
    print("    measure to the CENTRE OF THE WRIST ROLL AXIS.")
    mx = ask_float("      measured x (cm, forward from base origin): ")
    if mx is None:
        return None
    mz = ask_float("      measured z (cm, up from table): ")
    if mz is None:
        return None

    return {
        "ticks": ticks,
        "angles_rad": {k: round(v, 5) for k, v in rad.items()},
        "pred_cm": [round(px * 100, 2), round(py * 100, 2), round(pz * 100, 2)],
        "meas_cm": [round(mx, 2), round(mz, 2)],
        "err_cm": [round(mx - px * 100, 2), round(mz - pz * 100, 2)],
    }


def report(rows):
    """Print the comparison table and the verdict."""
    if not rows:
        print("\nNo measurements recorded.")
        return

    print("\n" + "=" * 68)
    print(f"{'pose':<6}{'pred x':>9}{'meas x':>9}{'err x':>8}"
          f"{'pred z':>9}{'meas z':>9}{'err z':>8}{'|err|':>8}")
    print("-" * 68)

    worst = 0.0
    total = 0.0
    for i, r in enumerate(rows, 1):
        ex, ez = r["err_cm"]
        mag = float(np.hypot(ex, ez))
        worst = max(worst, mag)
        total += mag
        print(f"{i:<6}{r['pred_cm'][0]:>9.1f}{r['meas_cm'][0]:>9.1f}{ex:>8.1f}"
              f"{r['pred_cm'][2]:>9.1f}{r['meas_cm'][1]:>9.1f}{ez:>8.1f}{mag:>8.1f}")

    mean = total / len(rows)
    print("-" * 68)
    print(f"mean error {mean:.2f} cm    worst {worst:.2f} cm    (target < {TARGET_CM} cm)")

    if worst < TARGET_CM:
        print("\n✅ FK validated.")
    else:
        print("\n❌ Off target. How the error behaves tells you which fault it is:")
        print("   - roughly constant in one direction  -> your x=0 mark is misplaced")
        print("   - grows as the arm extends           -> a joint zero is wrong")
        print("     (shoulder_pan and shoulder_lift had the largest span errors)")
        print("   - one pose bad, the rest fine        -> a mis-measurement; redo it")


def main():
    arm = Arm(FOLLOWER)
    arm.open()
    try:
        arm.load_calibration()

        poses = record_poses(arm)
        if not poses:
            print("No poses recorded — nothing to validate.")
            return

        print(f"\n--- PHASE 2: measure {len(poses)} poses ---")
        print("⚠️  Torque comes ON and the arm will move on its own. Hands clear.")
        for n in (3, 2, 1):
            print(f"  starting in {n}...")
            time.sleep(1)
        arm.torque(True)

        rows = []
        for i, ticks in enumerate(poses, 1):
            row = measure_pose(arm, ticks, i)
            if row is not None:
                rows.append(row)

        report(rows)

        if rows:
            os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
            with open(OUT_PATH, "w") as f:
                json.dump({"captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                           "target_cm": TARGET_CM,
                           "poses": rows}, f, indent=2)
            print(f"\nSaved -> {OUT_PATH}")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
