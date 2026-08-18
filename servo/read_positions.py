"""
Rung 1: live-print all 6 joint positions of the SO-101 follower.

Superseded by servo/arm.py for real work — kept as the first rung of the build
ladder. Redraws in place at 10 Hz until Ctrl-C.

    conda activate lerobot
    python servo/read_positions.py
"""

import scservo_sdk as scs
import time

# Pinned by USB-adapter serial: /dev/ttyACM* numbering reshuffles with plug order.
PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067297-if00"
BAUDRATE = 1000000
PRESENT_POSITION = 56   # register address, 2 bytes

JOINTS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6
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


LINES = len(JOINTS)
first = True
try:
    while True:
        # \033[nA moves the cursor up n lines so each pass overwrites the last
        # instead of scrolling; \033[K clears to end of line.
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
