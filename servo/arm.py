"""
servo/arm.py — the `Arm` class: ONE reusable interface to an SO-101 arm.

This consolidates what every scattered script re-implemented (open the port,
declare the joint map, byte-swap, torque, read/write) into a single class. The
endianness fix now lives in exactly ONE place (`_byteswap`), used by both reading
and writing.

Once you've filled in the TODOs:
    from servo.arm import Arm, FOLLOWER
    arm = Arm(FOLLOWER)     # makes an Arm object (calls __init__)
    arm.open()
    print(arm.read_joints())
    arm.torque(False)       # go limp
    arm.close()
"""

import scservo_sdk as scs

# ---- Constants (done for you — the same values your scripts used) ----
BAUDRATE = 1000000
PRESENT_POSITION = 56      # register we READ for current position
GOAL_POSITION    = 42      # register we WRITE to command a target
TORQUE_ENABLE    = 40      # write 0 = limp, 1 = stiff

# Stable by-id port paths — no more ttyACM roulette.
FOLLOWER = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067297-if00"
LEADER   = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067310-if00"

JOINTS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


def _byteswap(value):
    """The STS3215 endianness fix, in ONE place. Done for you.
    Swaps the low/high bytes of a 16-bit value. Used by BOTH read and write."""
    return ((value & 0xFF) << 8) | (value >> 8)


class Arm:
    # ------------------------------------------------------------------
    # TODO 1 — __init__: the "constructor". It runs once, automatically,
    #   when you write `Arm(FOLLOWER)`. `self` is THIS particular arm object;
    #   `port` is the device path passed in.
    #
    #   Store things on `self` so the other methods can reach them later:
    #       self.port = port
    #       self.port_handler = scs.PortHandler(port)
    #       self.packet_handler = scs.PacketHandler(1)
    #   (Create the handlers here, but don't open the port yet — that's open().)
    #
    #   `self.X = ...` is how an object REMEMBERS things between method calls.
    # ------------------------------------------------------------------
    def __init__(self, port):
        self.port = port
        self.port_handler = scs.PortHandler(port)
        self.packet_handler = scs.PacketHandler(1)

    # ---- port lifecycle ----
    def open(self):
        """Open the port + set baud. Done for you — note how it uses self.port_handler."""
        self.port_handler.openPort()
        self.port_handler.setBaudRate(BAUDRATE)

    def close(self):
        self.port_handler.closePort()

    # ---- the four real operations ----
    def torque(self, on):
        """Enable (on=True) / disable (on=False) holding torque on all joints.
        Done for you — study how it loops JOINTS and uses self.packet_handler."""
        for servo_id in JOINTS.values():
            self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, TORQUE_ENABLE, 1 if on else 0)

    def read_joints(self):
        # ------------------------------------------------------------------
        # TODO 3 — read all 6 joints; return a dict {name: position}.
        #   This is your teach.py read loop, reborn as a method. Loop JOINTS.items();
        #   for each (name, servo_id):
        #       raw, result, error = self.packet_handler.read2ByteTxRx(
        #           self.port_handler, servo_id, PRESENT_POSITION)
        #       if result != scs.COMM_SUCCESS: positions[name] = None
        #       else:                          positions[name] = _byteswap(raw)
        #   Build `positions` before the loop, return it after.
        # ------------------------------------------------------------------
        positions = {}
        for name, servo_id in JOINTS.items():
            raw, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, PRESENT_POSITION)
            if result != scs.COMM_SUCCESS: positions[name] = None
            else:                          positions[name] = _byteswap(raw)
        return positions
            

    def write_joints(self, targets):
        # ------------------------------------------------------------------
        # TODO 4 — command joints to targets. `targets` is a dict {name: position}
        #   (same shape read_joints returns). Loop targets.items(); for each:
        #       self.packet_handler.write2ByteTxRx(
        #           self.port_handler, JOINTS[name], GOAL_POSITION, _byteswap(position))
        #   Note: _byteswap on WRITE too — same fix, same one place.
        # ------------------------------------------------------------------
        for name, position in targets.items():
            self.packet_handler.write2ByteTxRx(
                self.port_handler, JOINTS[name], GOAL_POSITION, _byteswap(position))
        


if __name__ == "__main__":
    # Smoke test — run `python servo/arm.py` with the follower plugged in.
    arm = Arm(FOLLOWER)
    arm.open()
    try:
        print(arm.read_joints())
    finally:
        arm.close()
