"""
Rung 3: "Teach" — hand-move the follower arm and record the motion to a file.

How it works:
  1. We DISABLE motor torque so the arm goes limp and you can move it by hand.
  2. While you move it, we record all 6 joint positions ~10x/second into a list.
  3. When you press Ctrl-C, we save that list to a JSON file (replayed in Rung 4).

⚠️ Hold the arm up BEFORE running — it will sag under gravity when torque cuts.

Run (follower arm plugged in), from the repo root:
    conda activate lerobot
    python teach/teach.py
"""

import scservo_sdk as scs
import time
import json

# Follower arm, pinned by its USB-adapter serial so it never depends on plug
# order (ttyACM0/1 numbering reshuffles; this by-id path does not).
PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067297-if00"
BAUDRATE = 1000000
PRESENT_POSITION = 56
TORQUE_ENABLE = 40                       # write 0 = limp (movable), 1 = stiff (holds)
RECORDING_FILE = "/home/hanson/so-101-arm/data/traj_001.json"

JOINTS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

port_handler = scs.PortHandler(PORT)
packet_handler = scs.PacketHandler(1)
port_handler.openPort()
port_handler.setBaudRate(BAUDRATE)


def read_position(servo_id):
    """Read one servo's position (0-4095), byte-swap-corrected. Done for you."""
    position, result, error = packet_handler.read2ByteTxRx(
        port_handler, servo_id, PRESENT_POSITION
    )
    if result != scs.COMM_SUCCESS:
        return None
    return ((position & 0xFF) << 8) | (position >> 8)


def set_torque(enable):
    """Enable (True) or disable (False) holding torque on ALL motors. Done for you."""
    for servo_id in JOINTS.values():
        packet_handler.write1ByteTxRx(port_handler, servo_id, TORQUE_ENABLE, 1 if enable else 0)


# Make the arm limp so you can move it by hand.
set_torque(False)
print("Torque OFF — move the arm by hand. Press Ctrl-C to stop and save.\n")

frames = []   # each item will be one snapshot: {joint_name: position, ...}
start_time = time.time()

try:
    while True:
        # ------------------------------------------------------------------
        # TODO A — build ONE frame: a dict mapping each joint name to its
        #          current position. Loop over JOINTS, call read_position().
        #          (This is your Rung 1 loop, but storing into a dict instead
        #           of printing.)
        #
        #   If ANY reading came back None, skip this frame entirely so a bad
        #   read never pollutes the recording. Hint:
        #       if None in frame.values():
        #           continue        # jumps straight to the next while-loop pass
        #
        # TODO B — append the finished frame to the `frames` list.
        # ------------------------------------------------------------------
        # TODO A — build ONE complete frame first (all 6 joints), THEN decide.
        t = time.time() - start_time
        positions = {}
        for name, servo_id in JOINTS.items():
            positions[name] = read_position(servo_id)   # int, not a list

        # The frame is now complete. Check it once, append it once.
        if None in positions.values():
            continue          # a servo read failed → throw the whole frame away
        frames.append({"t": t, "positions": positions})  # good frame → record it (one append per snapshot)

        time.sleep(0.02)   # ~50 Hz sampling → dense waypoints → smooth replay
                           # (was 0.1 / 10 Hz, which made replay stop-start jitter)
except KeyboardInterrupt:
    print(f"\nRecorded {len(frames)} frames.")
finally:
    # ------------------------------------------------------------------
    # TODO C — save `frames` to RECORDING_FILE as JSON.
    #   Hint (ABS Ch 9 — file I/O):
    #       with open(RECORDING_FILE, "w") as f:
    #           json.dump(frames, f)
    #   The `with` block opens the file and auto-closes it when done.
    # ------------------------------------------------------------------
    with open(RECORDING_FILE, "w") as f:
        json.dump(frames, f)
    port_handler.closePort()
