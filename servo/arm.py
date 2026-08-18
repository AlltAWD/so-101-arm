"""
servo/arm.py — the `Arm` class: one reusable interface to an SO-101 arm.

Consolidates what every earlier script re-implemented (port setup, joint map,
byte-swap, torque, read/write) into a single class. The STS3215 endianness fix
lives in exactly one place, `_byteswap`, used by both reading and writing.

    from servo.arm import Arm, FOLLOWER
    arm = Arm(FOLLOWER)
    arm.open()
    print(arm.read_joints())
    arm.torque(False)      # go limp
    arm.close()
"""

import scservo_sdk as scs
import numpy as np
import json
import os
import time

BAUDRATE = 1000000
PRESENT_POSITION = 56      # register we READ for current position
GOAL_POSITION    = 42      # register we WRITE to command a target
TORQUE_ENABLE    = 40      # write 0 = limp, 1 = stiff

TICKS_PER_REV = 4096                       # STS3215: 4096 encoder counts / revolution
RAD_PER_TICK  = 2 * np.pi / TICKS_PER_REV  # ~0.001534 rad per tick

# Pinned by USB-adapter serial: /dev/ttyACM* numbering reshuffles with plug order.
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
    """Swap the low/high bytes of a 16-bit value — the STS3215 endianness fix."""
    return ((value & 0xFF) << 8) | (value >> 8)


class Arm:
    def __init__(self, port):
        """Set up handlers for `port`. The port itself isn't opened until open()."""
        self.port = port
        self.port_handler = scs.PortHandler(port)
        self.packet_handler = scs.PacketHandler(1)
        self.calib = None   # filled by load_calibration(); {name: {offset, sign}}

    # ---- port lifecycle ----
    def open(self):
        """Open the serial port and set the baud rate."""
        self.port_handler.openPort()
        self.port_handler.setBaudRate(BAUDRATE)

    def close(self):
        """Close the serial port."""
        self.port_handler.closePort()

    # ---- the four real operations ----
    def torque(self, on):
        """Enable (on=True) or disable holding torque on all joints."""
        for servo_id in JOINTS.values():
            self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, TORQUE_ENABLE, 1 if on else 0)

    def read_joints(self):
        """All 6 joint positions in ticks as {name: position}.

        A joint whose read fails comes back None rather than raising, so a single
        comm hiccup doesn't kill a recording loop.
        """
        positions = {}
        for name, servo_id in JOINTS.items():
            raw, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, PRESENT_POSITION)
            if result != scs.COMM_SUCCESS: positions[name] = None
            else:                          positions[name] = _byteswap(raw)
        return positions


    def write_joints(self, targets):
        """Command joints to `targets`, a {name: position} dict in ticks.

        Writes need the same byte-swap as reads.
        """
        for name, position in targets.items():
            self.packet_handler.write2ByteTxRx(
                self.port_handler, JOINTS[name], GOAL_POSITION, _byteswap(position))

    # ---- calibration: ticks <-> radians ----
    def load_calibration(self, path=None):
        """Load per-joint {offset, sign} produced by servo/calibrate.py.

        Call once after open(). Defaults to data/calib.json at the repo root.
        """
        if path is None:
            here = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(os.path.dirname(here), "data", "calib.json")
        with open(path) as f:
            self.calib = json.load(f)["joints"]
        return self.calib

    def ticks_to_rad(self, name, ticks):
        """One joint's raw ticks -> radians about its calibrated zero."""
        c = self.calib[name]
        return c["sign"] * (ticks - c["offset"]) * RAD_PER_TICK

    def rad_to_ticks(self, name, rad):
        """Inverse of ticks_to_rad — radians -> the raw tick to command."""
        c = self.calib[name]
        return int(round(c["offset"] + c["sign"] * rad / RAD_PER_TICK))

    def read_joints_rad(self):
        """read_joints(), converted to radians. Requires load_calibration() first."""
        if self.calib is None:
            raise RuntimeError("Call load_calibration() before read_joints_rad().")
        raw = self.read_joints()
        return {name: (None if raw[name] is None else self.ticks_to_rad(name, raw[name]))
                for name in raw}

    # ---- smooth motion ----
    def goto(self, targets, ticks_per_step=10, dt=0.02):
        """Move smoothly from the current pose to `targets`, hop by hop.

        A bare write_joints() slams one goal position at the servo, so it lurches.
        Here we read where we are now and walk to the target in small increments.

        Smoothness lives in the DISTANCE per hop, not the hop count: too fine and
        the servo finishes each hop instantly then idles (jitter); too coarse and
        it lurches. So the step count is derived from how far the farthest joint
        travels, keeping every hop about `ticks_per_step`. Big and small moves
        then flow the same way — a single fixed speed cannot serve both.

            arm.goto({"shoulder_pan": 2048}, ticks_per_step=10)

        `targets` may list only some joints; only those move.
        """
        start = self.read_joints()
        max_dist = max(abs(targets[name] - start[name]) for name in targets)
        steps = max(2, int(max_dist / ticks_per_step))

        paths = {name: np.linspace(start[name], target, steps)
                for name, target in targets.items()}
        for i in range(steps):
            frame = {name: int(paths[name][i]) for name in targets}
            self.write_joints(frame)
            time.sleep(dt)


if __name__ == "__main__":
    # Smoke test — run with the follower plugged in. If data/calib.json exists,
    # also prints radians: at the calibrated zero every joint should read ~0.0.
    arm = Arm(FOLLOWER)
    arm.open()
    try:
        print("ticks:", arm.read_joints())
        try:
            arm.load_calibration()
            rad = arm.read_joints_rad()
            print("rad:  ", {k: (None if v is None else round(v, 4)) for k, v in rad.items()})
        except FileNotFoundError:
            print("(no data/calib.json yet — run: python servo/calibrate.py)")
    finally:
        arm.close()
