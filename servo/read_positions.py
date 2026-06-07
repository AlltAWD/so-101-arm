"""
Rung 1: Read and print all 6 joint positions of the SO-101 follower arm.

Goal: when you run this, it prints something like
    shoulder_pan   :  2048
    shoulder_lift  :  1730
    ...
    gripper        :  2600

Run it (with the FOLLOWER arm plugged in) using:
    conda activate lerobot
    python ~/read_positions.py

The hardware boilerplate is done for you. Fill in the 3 TODOs below —
those are the Python parts you're practicing.
"""

import scservo_sdk as scs
import time

PORT = "/dev/ttyACM1"   # follower arm (leader is ttyACM0)
BAUDRATE = 1000000
PRESENT_POSITION = 56   # register address; 2 bytes (you found this in tables.py)

# ----------------------------------------------------------------------
# TODO 1 — a dictionary mapping each joint NAME to its servo ID.
# The SO-101 has 6 joints, IDs 1 through 6, in this order:
#   shoulder_pan=1, shoulder_lift=2, elbow_flex=3,
#   wrist_flex=4, wrist_roll=5, gripper=6
# Build that dict. (Practicing: dictionaries)
JOINTS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6
}
# ----------------------------------------------------------------------

# Hardware setup — done for you.
port_handler = scs.PortHandler(PORT)
packet_handler = scs.PacketHandler(1)
port_handler.openPort()
port_handler.setBaudRate(BAUDRATE)


def read_position(servo_id):
    """Read the current position (0-4095) of one servo. Done for you."""
    position, result, error = packet_handler.read2ByteTxRx(
        port_handler, servo_id, PRESENT_POSITION
    )
    if result != scs.COMM_SUCCESS:
        return None  # couldn't read this one
    return ((position & 0xFF) << 8) | (position >> 8)


# ----------------------------------------------------------------------
# TODO 2 — loop over every joint in JOINTS, read its position,
# and print it. (Practicing: looping over a dict, calling a function)
#
# Hint: `for name, servo_id in JOINTS.items():`
# Inside the loop, call read_position(servo_id) and print the result.
#
# TODO 3 — make the print line up nicely, e.g.
#   print(f"{name:14}: {pos}")
# The `:14` pads the name to 14 characters. (Practicing: f-strings)
# ----------------------------------------------------------------------

# <-- your loop goes here
LINES = len(JOINTS)
first = True
try:
    while True:
        if not first:
            print(f"\033[{LINES}A", end="")
        first = False
        for name, servo_id in JOINTS.items():
            print(f"{name:14}: {read_position(servo_id):>5}\033[K")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopping.")
finally:        
    port_handler.closePort()
