# SO-101 Manipulator — Learning Robotics from Servos to Policies

A self-directed project teaching myself robotics on a low-cost open-source arm
(SO-101 / [HuggingFace LeRobot](https://github.com/huggingface/lerobot)), built
from low-level servo control up to a learned visuomotor policy.

**Author:** Hanson ([@AlltAWD](https://github.com/AlltAWD)) · Physics, Grinnell College
**Hardware:** SO-101 leader/follower arms · Feetech STS3215 serial-bus servos
**Stack:** Python · `scservo_sdk` · LeRobot

---

## Status / roadmap
Built bottom-up; each item links to its script + a short writeup.

- [x] **Servo tooling** — scan bus IDs, fix duplicate IDs, read live joint positions
- [ ] **Teach-and-repeat** — record joint trajectories to JSON, then replay
- [ ] **Forward kinematics** — predict end-effector pose, validated against measurement
- [ ] **Servo characterization** — STS3215 accuracy / latency / sag-under-load
- [ ] **LeRobot dataset** — teleoperated demonstrations for a pick task
- [ ] **Inverse kinematics + PID controller** — closed-loop to target poses
- [ ] **Imitation-learning demo** — ACT policy performing the task autonomously
- [ ] **(stretch) Gravity compensation**

## Repo layout
```
servo/        # low-level: scan_servos.py, fix_id.py, read_positions.py
teach/        # record + replay (teach-and-repeat)
kinematics/   # fk.py, ik.py
control/      # PID controller, characterization scripts
data/         # recorded datasets (gitignored if large)
docs/         # writeups, plots, demo videos
```

## Notes / gotchas
- **STS3215 endianness:** raw `read2ByteTxRx` returns byte-swapped values;
  correct with `((p & 0xFF) << 8) | (p >> 8)`.
- Leader = `/dev/ttyACM0`, Follower = `/dev/ttyACM1`; LeRobot v0.4.5, conda env `lerobot`.

## Setup
```bash
conda activate lerobot
python servo/read_positions.py   # live-prints all 6 follower joint positions
```
