# SO-101 Manipulator — Learning Robotics from Servos Up

A self-directed project teaching myself robotics on a low-cost open-source arm
(SO-101 / [HuggingFace LeRobot](https://github.com/huggingface/lerobot)), built
from raw serial-bus servo commands upward. Everything here is written from
scratch rather than driven through LeRobot's stock pipeline — the point is to
understand each layer, not to run a demo.

**Author:** Hanson ([@AlltAWD](https://github.com/AlltAWD)) · Physics, Grinnell College
**Hardware:** SO-101 leader/follower arms · Feetech STS3215 serial-bus servos
**Stack:** Python · `scservo_sdk` · NumPy · LeRobot v0.4.5

---

## Status

- [x] **Servo tooling** — scan bus IDs, reassign a duplicate ID, live-read joint positions
- [x] **Teach-and-repeat** — record joint trajectories to JSON and replay them
- [x] **`Arm` class** — one reusable hardware interface; smooth interpolated moves
- [x] **Tick↔radian calibration** — joint zeros derived from measured mechanical range
- [x] **Forward kinematics** — validated against ruler measurement to 0.4 cm RMS
- [ ] **Servo characterization** — STS3215 accuracy / latency / sag under load
- [ ] **PID joint controller** — closed-loop to a target angle
- [ ] **Inverse kinematics**
- [ ] **LeRobot dataset** — teleoperated demonstrations for a pick task
- [ ] **Imitation-learning demo** — ACT policy performing the task autonomously

## Result: forward kinematics, validated

FK predicts the wrist-roll frame position from joint angles, chaining one
homogeneous transform per joint from the CAD-derived URDF parameters. Checked
against ruler measurements at four poses spanning 14–40 cm of reach:

| | |
|---|---|
| Mean absolute error | 0.8 cm |
| Systematic bias | +0.27 cm in x, +0.66 cm in z |
| Residual scatter after removing bias | **0.37 cm (x), 0.44 cm (z)** RMS |
| Correlation of error with reach | **−0.94** |

The constant z bias is a datum-placement error — the measurement reference sat
~7 mm below the URDF's base plane — not a model error. What matters is the last
row: a wrong joint zero is an *angular* error, so it would be amplified by the
lever arm and grow with reach. Here the error *shrinks* with reach (the two most
extended poses are the two most accurate), which rules out joint-zero error as a
contributor. Residual scatter of ~0.4 cm is consistent with the resolution of
hand measurement — locating the wrist-roll axis by eye is itself ±0.3–0.5 cm.

## Repo layout

```
servo/
  arm.py            # the Arm class: port, torque, read/write, ticks<->radians, goto()
  calibrate.py      # derive each joint's zero from its measured mechanical range
  scan_servos.py    # ping IDs 1-6 on the bus
  fix_id.py         # reassign a servo's ID (used to fix a duplicate)
  read_positions.py # rung 1: live-print all six joints
  torque_off.py     # limp both arms on demand
teach/
  teach.py          # record joint trajectories at 50 Hz to JSON
  repeat.py         # replay them at recorded timing
  diag.py           # torque-release + live-read check
kinematics/
  urdf_params.py    # link geometry, verbatim from the CAD-generated URDF
  fk.py             # homogeneous-transform chain: joint angles -> end pose
data/               # calibration, recorded trajectories, validation results
ros2/
  so101_ros/        # ROS 2 (Jazzy) ament_python package, built from ~/ros2_ws via a symlink
    so101_ros/      #   nodes — stdMsgPublisher.py: first 1 Hz String publisher
    urdf/           #   two_link.urdf: minimal 2-link practice model
notes/
  ros2-cli.md       # what nodes, topics and messages are
```

## Notes / gotchas

Things that cost real debugging time, recorded so they don't cost it twice.

- **STS3215 endianness.** Raw `read2ByteTxRx` returns byte-swapped values; correct
  with `((p & 0xFF) << 8) | (p >> 8)`. Writes need the same swap. In this repo the
  fix lives in exactly one place, `Arm._byteswap`.
- **`/dev/ttyACM*` numbering is not stable.** It reshuffles with USB enumeration
  order, which caused an afternoon of "the wrong arm is moving." Both arms are
  pinned by USB-adapter serial via `/dev/serial/by-id/...` instead. Symptom of a
  mixup: a recording where every joint has zero span, because you're reading the
  arm you aren't moving.
- **Replay smoothness is distance-per-hop, not hop count.** Tiny interpolation
  steps make the servo finish each hop instantly then idle until the next write,
  which reads as jitter. `Arm.goto()` derives its step count from travel distance
  so every hop is ~10 ticks, and small and large moves glide alike. A single fixed
  speed cap can't serve both.
- **Sampling rate matters for replay.** Recording at 10 Hz left gaps wide enough
  that the servos visibly stop-start between waypoints; 50 Hz fixed it.
- **Calibration zero must match the URDF's convention.** The *new_calib* URDF
  defines joint zero as the *middle of each joint's range*, not a straight or
  extended pose. An earlier hand-posed "straight home" reference was off by ~12°
  on two joints — FK was correct and disagreed with reality anyway. `calibrate.py`
  now derives the zero from two mechanical-stop captures per joint, and
  cross-checks the measured range against the URDF's declared range so a bad
  capture is caught at the arm rather than three steps later.

## Setup

```bash
conda activate lerobot

python servo/scan_servos.py       # confirm all six servos answer
python servo/read_positions.py    # live-print follower joint positions
python servo/calibrate.py         # derive joint zeros (writes data/calib.json)
python kinematics/fk.py           # FK self-checks
```

Teach and replay a motion:

```bash
python teach/teach.py             # torque off; hand-move the arm, Ctrl-C to save
python teach/repeat.py            # torque on; replays the recording
```

### ROS 2

ROS 2 Jazzy runs on the system Python — **don't `conda activate` in a ROS terminal.** The package
lives in this repo; `~/ros2_ws` is only a build workspace that links to it:

```bash
mkdir -p ~/ros2_ws/src
ln -s ~/so-101-arm/ros2/so101_ros ~/ros2_ws/src/so101_ros
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 run so101_ros talker         # 1 Hz "Hello World" on /topic
```
