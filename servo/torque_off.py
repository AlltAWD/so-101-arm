"""
Disable holding torque on BOTH arms so they go limp (back-drivable by hand).

⚠️ The arm SAGS under gravity the instant torque cuts — support it first, or
   move it to a low/resting pose before running.

Run from the repo root:
    conda activate lerobot
    python servo/torque_off.py
"""

import scservo_sdk as scs

BAUDRATE = 1000000
TORQUE_ENABLE = 40                       # write 0 = limp, 1 = stiff
IDS = [1, 2, 3, 4, 5, 6]

# Both arms, pinned by USB-adapter serial (stable across plug order).
PORTS = {
    "follower": "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067297-if00",
    "leader":   "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067310-if00",
}

for name, dev in PORTS.items():
    port = scs.PortHandler(dev)
    ph = scs.PacketHandler(1)
    if not port.openPort():
        print(f"{name}: could not open {dev} (unplugged?) — skipping")
        continue
    port.setBaudRate(BAUDRATE)
    for sid in IDS:
        ph.write1ByteTxRx(port, sid, TORQUE_ENABLE, 0)
    port.closePort()
    print(f"{name}: torque OFF (limp)")
