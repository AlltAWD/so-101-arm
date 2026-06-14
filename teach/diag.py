"""
Diagnostic: is the follower actually going limp, and do reads track movement?

Safe to run — it only DISABLES torque and READS. It never commands motion.
While it runs, gently hand-move a couple of joints and watch the numbers.
Ctrl-C to stop.
"""

import scservo_sdk as scs
import time

# Follower arm, pinned by USB-adapter serial (stable across plug order).
PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067297-if00"
BAUDRATE = 1000000
PRESENT_POSITION = 56
TORQUE_ENABLE = 40

JOINTS = {
    "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
    "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
}

port = scs.PortHandler(PORT)
ph = scs.PacketHandler(1)
port.openPort()
port.setBaudRate(BAUDRATE)

# Disable torque AND report whether each write actually lands.
print("Disabling torque (checking comm result per servo):")
for name, sid in JOINTS.items():
    result, error = ph.write1ByteTxRx(port, sid, TORQUE_ENABLE, 0)
    ok = "OK" if result == scs.COMM_SUCCESS else f"FAIL ({ph.getTxRxResult(result)})"
    print(f"  id {sid} {name:14s} torque-off: {ok}")

print("\nNow MOVE the arm by hand. Watching positions (Ctrl-C to stop):\n")
try:
    while True:
        row = []
        for name, sid in JOINTS.items():
            pos, result, error = ph.read2ByteTxRx(port, sid, PRESENT_POSITION)
            if result != scs.COMM_SUCCESS:
                row.append(f"{name}=ERR")
            else:
                pos = ((pos & 0xFF) << 8) | (pos >> 8)
                row.append(f"{name}={pos}")
        print("  " + "  ".join(row))
        time.sleep(0.2)
except KeyboardInterrupt:
    print("\nstopped.")
finally:
    port.closePort()
