# Vision-based Pick & Place on ROS 2 + robosuite

Using **monocular camera images only**, this demo detects 3 balls on a table and tidies them into a box via pick & place, running as a multi-node ROS 2 system.

```
image → HSV detection + intersection with the table plane for 3D → FSM for pick & place
```

The simulator's ground truth (GT) is **never used for control**. The separation of perception and control is demonstrated in a form close to a real robot setup.

---

## Index

1. [Demo](#demo)
2. [Overview](#overview)
3. [Design Highlights](#design-highlights)
4. [Architecture](#architecture)
5. [How It Works](#how-it-works)
6. [Configuration (`config/`)](#configuration-config)
7. [Running the Demo](#running-the-demo)
8. [Tech Stack](#tech-stack)

---

## Demo
https://github.com/user-attachments/assets/3203123b-01db-4fe1-a728-46dfe3c75d12

---

## Overview

| Item | Description |
|---|---|
| Task | Put 3 orange balls on the table into a blue box |
| Sensor | Monocular RGB (agentview, no depth) |
| Robot | Panda + OSC_POSE (robosuite / MuJoCo) |
| Software | ROS 2 Humble, 3 nodes (+ optional error evaluation) |
| Control input | Perception estimates `/percept/*` only (GT is for debugging only) |
| Parameters | Consolidated in `config/*.json` (tunable without code changes) |

The repository name contains "LIBERO", but this demo is a custom tidy-up environment built on robosuite.

---

## Design Highlights

1. **Node split assuming a real robot**
   Simulation boundary / perception / task control run as separate processes, connected via topics. The design avoids relying on convenient ground truth available inside the simulator.

2. **Separate topics for perception and ground truth**
   - Control: `/percept/ball_poses`, `/percept/box_pose`
   - Evaluation only: `/gt/*`
   This makes it possible to "drive with vision" and "measure the error" at the same time.

3. **Explicit assumptions for monocular → 3D**
   With no depth available, (u, v) is lifted to 3D by intersecting a camera ray with a horizontal plane at a known table height. Assumptions such as the ball radius are shared with `config/env.json`, making the impact of any mismatch easy to trace.

4. **Axis-decoupled state machine**
   XY and Z motions are never mixed; grasp and release use zero translation plus a gripper command only. To suppress false transitions caused by overshoot, a step advances only after the target is reached within tolerance for consecutive cycles.

5. **Externalized configuration**
   Scene, HSV, and control gains live in JSON. Reproducing experiments and tuning are decoupled from the code.

---

## Architecture

Data flows in essentially one direction; only robot commands travel back to the Bridge.

```mermaid
flowchart TD
    SIM["MuJoCo / robosuite<br/>CustomTidyUpEnv"]

    BR["<b>robosuite_bridge_node</b><br/>sim boundary · 40 Hz"]
    HSV["<b>hsv_perceptor_node</b><br/>HSV → 3D"]
    TM["<b>task_manager_node</b><br/>FSM · 40 Hz"]
    DBG["percept_error_debug_node<br/><i>optional · error evaluation</i>"]

    SIM <-->|"step / obs"| BR
    BR -->|"image / K / extrinsic / table_z"| HSV
    HSV -->|"/percept/*"| TM
    BR -->|"/robot/eef_pose"| TM
    TM -->|"/robot/cmd_action"| BR
    BR -.->|"/gt/*"| DBG
    HSV -.->|"/percept/*"| DBG

    classDef main fill:#e8f0fe,stroke:#4a76c8,stroke-width:2px
    classDef opt fill:#f5f5f5,stroke:#aaa,stroke-dasharray:4 3
    class BR,HSV,TM main
    class DBG opt
```

### Key files

| File | Role |
|---|---|
| `robosuite_bridge_node.py` | The only boundary that touches robosuite. Publishes observations, applies actions |
| `hsv_perceptor_node.py` | Color detection and 3D estimation via table-plane intersection |
| `task_manager_node.py` | Pick & place state machine |
| `percept_error_debug_node.py` | Error between `/percept` and `/gt` (optional) |
| `config_loader.py` / `config/` | Environment, perception, and control parameters |

### Key topics (control path)

| Topic | Direction | Contents |
|---|---|---|
| `/camera/image_raw` and others | Bridge → HSV | Image, intrinsics, extrinsics, table height |
| `/percept/ball_poses`, `/percept/box_pose` | HSV → Task Manager | Estimated positions |
| `/robot/eef_pose` | Bridge → Task Manager | End-effector position |
| `/robot/cmd_action` | Task Manager → Bridge | 7-dimensional OSC (translation + gripper) |

Ground truth `/gt/*` and errors `/debug/*` are for evaluation and never enter the control loop.

---

## How It Works

### Perception

1. Mask the balls (orange) and the box (blue) in HSV
2. Take the centroid (u, v) of each contour
3. Cast a ray from the camera and intersect it with a horizontal plane at a known height to get 3D
   - Balls: `table_height + ball_radius`
   - Box: `table_height + box_center_offset_z`

### Control (axis-decoupled FSM)

Once detection starts, the ball and box positions are latched, and the following is repeated for each ball (the travel height is the shared `travel_z`).

```
0  Z → travel_z
1  XY → directly above the ball (height fixed)
2  Z → ball center
3  grasp (close gripper, zero translation)
4  Z → travel_z
5  XY → directly above the box
6  release (open gripper, zero translation) → next ball
```

OSC commands are normalized to `[-1, 1]`. Commands are limited by a P gain and distance-dependent clipping.

---

## Configuration (`config/`)

| File | Scope | Examples |
|---|---|---|
| `config/env.json` | Scene / sim | Table height, ball position/radius/friction, box shape, camera, `control_freq=40`, `action_alpha` |
| `config/perception.json` | Perception | HSV thresholds, minimum area |
| `config/control.json` | Control | `travel_z`, tolerances, `kp`, hold steps |

`sim.control_freq` and `control_hz` are expected to match (40 Hz by default).

---

## Running the Demo

Three terminals, in this order.

```bash
# Activate the environment in each terminal first
python robosuite_bridge_node.py      # A: simulation
python hsv_perceptor_node.py         # B: perception (with debug window)
python task_manager_node.py          # C: control
# Optional: python percept_error_debug_node.py
```

When everything works, the control side prints `Locked 3 balls + box; starting pick sequence` and motion begins.

<details>
<summary><b>Environment setup (macOS / Ubuntu)</b></summary>

### Prerequisites

- macOS arm64 (conda + [RoboStack](https://robostack.github.io/)) or Ubuntu 22.04 (ROS 2 Humble via apt)
- Main dependencies: `rclpy`, `cv_bridge`, `robosuite==1.4.1`, `mujoco==3.1.2`, OpenCV, NumPy

### macOS (Apple Silicon)

```bash
conda create -y -n ros2_libero --solver=libmamba --override-channels \
  -c conda-forge -c robostack-staging \
  python=3.11 "numpy=1.26" \
  ros-humble-ros-base ros-humble-cv-bridge \
  numba scipy pillow py-opencv glfw

conda activate ros2_libero
pip install "mujoco==3.1.2" pynput termcolor
pip install --no-deps "robosuite==1.4.1"
```

- `--solver=libmamba` / `--override-channels` are required for RoboStack resolution
- `robosuite==1.4.1` is pinned (the 1.5 series is API-incompatible)
- On macOS, do not install pip's `opencv-python` (hence `--no-deps`)

### Ubuntu 22.04

```bash
sudo apt install ros-humble-ros-base ros-humble-cv-bridge
pip install "mujoco==3.1.2" "robosuite==1.4.1" "numpy<2" opencv-python glfw pynput termcolor
source /opt/ros/humble/setup.bash
```

### Verify the installation

```bash
python -c "import rclpy, cv_bridge, robosuite, mujoco, cv2, numpy; print('ok')"
```

</details>

<details>
<summary><b>Troubleshooting (summary)</b></summary>

| Symptom | What to check |
|---|---|
| `No module named 'rclpy'` | Whether the conda / ROS environment is activated |
| `load_controller_config` missing | Revert to `robosuite==1.4.1` |
| Balls are not detected | The perception window and the HSV / area values in `perception.json` |
| The arm does not move | All 3 nodes running, `/percept/ball_poses` being published, the `Locked 3 balls` log |
| Grasping fails | `z_tol` / `grasp_hold_steps` in `control.json`, ball radius in `env.json` |

On macOS, do not move MuJoCo's GL off the main thread (`sim.mujoco_gl` should be `glfw` / `cgl`).

</details>

---

## Tech Stack

ROS 2 Humble · Python · robosuite 1.4.1 · MuJoCo 3.1.2 · OpenCV · NumPy
