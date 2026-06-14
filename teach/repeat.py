"""
Rung 4: "Repeat" — load a taught trajectory and replay it on the follower arm.

This is the other half of teach-and-repeat. teach.py recorded the arm being
hand-moved into data/traj_001.json; this script reads that file back and drives
the motors through the same positions, in the same timing.

How it works:
  1. We LOAD the recorded frames from the JSON file.
  2. We ENABLE motor torque so the arm can hold and move to commanded positions.
  3. We step through the frames, sending each joint to its recorded position,
     sleeping between frames so playback runs at the speed it was recorded.

⚠️ SAFETY — read before running:
  - Clear the space around the arm. It WILL move on its own.
  - Position the arm by hand near where you STARTED teaching, and keep a hand
    near the e-stop / power before the countdown ends. The first frame can jump.
  - First runs: keep taught motions small and slow. Tune smoothness later.

Run (follower arm plugged in), from the repo root:
    conda activate lerobot
    python teach/repeat.py
"""

import scservo_sdk as scs
import time
import json

# Follower arm, pinned by its USB-adapter serial so it never depends on plug
# order (ttyACM0/1 numbering reshuffles; this by-id path does not).
PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067297-if00"
BAUDRATE = 1000000
GOAL_POSITION = 42                       # register we WRITE to command a target (0-4095)
TORQUE_ENABLE = 40                       # write 0 = limp, 1 = stiff (holds + can be driven)
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


def write_position(servo_id, value):
    """Command one servo to a target position (0-4095). Done for you.

    Note the SAME byte-swap gotcha as reading: the STS3215 wants the bytes in
    the opposite order from what the SDK sends, so we swap before writing.
    """
    swapped = ((value & 0xFF) << 8) | (value >> 8)
    packet_handler.write2ByteTxRx(port_handler, servo_id, GOAL_POSITION, swapped)


def set_torque(enable):
    """Enable (True) or disable (False) holding torque on ALL motors. Done for you."""
    for servo_id in JOINTS.values():
        packet_handler.write1ByteTxRx(port_handler, servo_id, TORQUE_ENABLE, 1 if enable else 0)


# ----------------------------------------------------------------------------
# TODO A — load the recording.
#   teach.py did:  json.dump(frames, f)   to SAVE.
#   You want the mirror image: OPEN RECORDING_FILE for reading ("r") and use
#   json.load(f) to get the list of frames back. Store it in `frames`.
#       with open(RECORDING_FILE, "r") as f:
#           frames = json.load(f)
#   Each frame looks exactly like what teach.py saved:
#       {"t": 1.23, "positions": {"shoulder_pan": 2048, "shoulder_lift": 1900, ...}}
# ----------------------------------------------------------------------------
with open(RECORDING_FILE, "r") as f:
    frames = json.load(f)   # <-- replace this with the loaded data (TODO A)

if not frames:
    print("No frames loaded. Run teach/teach.py first to record a trajectory.")
    port_handler.closePort()
    raise SystemExit

print(f"Loaded {len(frames)} frames. Position the arm near the START, hands clear.")
for n in (3, 2, 1):
    print(f"  replaying in {n}...")
    time.sleep(1)

set_torque(True)   # motors now hold + obey position commands

try:
    previous_t = frames[0]["t"]
    for frame in frames:
        # --------------------------------------------------------------------
        # TODO B — drive each joint to its recorded position.
        #   `frame["positions"]` is a dict {joint_name: position}. Loop over its
        #   .items() and call write_position(servo_id, position). You'll need the
        #   servo_id from JOINTS, e.g.  write_position(JOINTS[name], position).
        #
        # TODO C — match the recorded timing.
        #   Each frame has a timestamp `frame["t"]` (seconds since teaching began).
        #   Sleep for the GAP between this frame and the previous one so playback
        #   runs at record speed:
        #       dt = frame["t"] - previous_t
        #       time.sleep(max(0, dt))     # max(0, ...) guards against tiny negatives
        #   then update previous_t = frame["t"] for the next pass.
        # --------------------------------------------------------------------
        for name, position in frame["positions"].items():
            write_position(JOINTS[name], position)
        dt = frame["t"] - previous_t
        time.sleep(max(0, dt))
        previous_t = frame["t"]

    print("Replay complete.")
except KeyboardInterrupt:
    print("\nStopped early.")
finally:
    # Leave torque ON so the arm holds its final pose instead of flopping down.
    # (If you'd rather it go limp at the end, call set_torque(False) here.)
    port_handler.closePort()
