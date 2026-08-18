"""
Rung 3: "Teach" — hand-move the follower arm and record the motion to JSON.

Torque is disabled so the arm is back-drivable; all 6 joint positions are sampled
at 50 Hz while you move it, and Ctrl-C writes the trajectory out for teach/repeat.py.

⚠️ Hold the arm up BEFORE running — it will sag under gravity when torque cuts.

Run from the repo root:
    conda activate lerobot
    python teach/teach.py
"""

import scservo_sdk as scs
import time
import json

# Pinned by USB-adapter serial: /dev/ttyACM* numbering reshuffles with plug order.
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
    """One servo's position in ticks (0-4095), or None if the read failed.

    The STS3215 returns the two bytes swapped, hence the shift-and-mask.
    """
    position, result, error = packet_handler.read2ByteTxRx(
        port_handler, servo_id, PRESENT_POSITION
    )
    if result != scs.COMM_SUCCESS:
        return None
    return ((position & 0xFF) << 8) | (position >> 8)


def set_torque(enable):
    """Enable (True) or disable holding torque on all motors."""
    for servo_id in JOINTS.values():
        packet_handler.write1ByteTxRx(port_handler, servo_id, TORQUE_ENABLE, 1 if enable else 0)


set_torque(False)
print("Torque OFF — move the arm by hand. Press Ctrl-C to stop and save.\n")

frames = []   # each item is one snapshot: {"t": seconds, "positions": {name: ticks}}
start_time = time.time()

try:
    while True:
        t = time.time() - start_time
        positions = {}
        for name, servo_id in JOINTS.items():
            positions[name] = read_position(servo_id)

        # Build the whole frame first, then accept or reject it as a unit — a
        # single failed servo read would otherwise leave a hole in the recording.
        if None in positions.values():
            continue
        frames.append({"t": t, "positions": positions})

        time.sleep(0.02)   # 50 Hz — dense waypoints. At 10 Hz the gaps between
                           # waypoints made the servos stop-start during replay.
except KeyboardInterrupt:
    print(f"\nRecorded {len(frames)} frames.")
finally:
    with open(RECORDING_FILE, "w") as f:
        json.dump(frames, f)
    port_handler.closePort()
