# Autonomous Mobile Manipulation in Simulation

A ROS 2 system where a mobile manipulator autonomously interprets natural language task prompts and executes pick-and-place operations inside a simulated three-room environment. The robot navigates between rooms, identifies objects, grasps them with a robot arm, and returns them to a base station — all driven from a single text instruction.

**Maintainer:** Leonardo Andreae — leoandreae@gmail.com

---

## Robot Platform

**Clearpath Ridgeback + Franka FR3**

| Component | Details |
|---|---|
| Mobile base | Clearpath Ridgeback — omnidirectional (holonomic), 4-wheel mecanum drive |
| Arm | Franka FR3 — 7 DoF, torque-controlled |
| Gripper | Franka Hand — parallel gripper with force/torque sensing |
| Sensors | 2D LiDAR, wrist RGB-D camera (D435i-class), IMU |

The holonomic base allows precise positioning relative to objects without multi-step turning manoeuvres. The combined URDF is in `amm_description/urdf/ridgeback_franka.urdf.xacro`.

---

## Simulators

The system supports two simulator backends, selected at launch time:

### Gazebo Harmonic (default — CPU only, no GPU required)

Recommended for development on laptops or machines without an NVIDIA GPU.

- **Simulator:** Gazebo Harmonic (`gz-sim8`)
- **SLAM:** `slam_toolbox` (2D LiDAR, CPU)
- **Object detection:** Direct model-pose lookup via TF/ros_gz_bridge — ground-truth poses, no vision inference
- **Motion planning:** OMPL (CPU)

### NVIDIA Isaac Sim (GPU — requires NVIDIA GPU + CUDA 12.x)

For high-fidelity rendering and GPU-accelerated perception/planning.

- **Simulator:** NVIDIA Isaac Sim (Omniverse)
- **SLAM:** `slam_toolbox` or `isaac_ros_visual_slam`
- **Object detection:** `isaac_ros_grounding_dino` (open-vocabulary 2D) + `isaac_ros_foundationpose` (6-DoF pose)
- **Motion planning:** `isaac_ros_cumotion` (CUDA-accelerated)

---

## Middleware

| Layer | Technology |
|---|---|
| Middleware | ROS 2 Jazzy Jalisco (C++) |
| Build system | colcon / CMake |
| SLAM | slam_toolbox (2D LiDAR, CPU) |
| Navigation | Nav2 (MPPI controller, Navfn planner, layered costmaps) |
| Manipulation | MoveIt 2 + MoveIt Task Constructor (MTC) |
| LLM | Ollama — local, free, no API key (model: Llama 3.2) |

---

## System Architecture

Natural language flows through a chain of ROS 2 action servers:

```
User prompt
    │
    ▼
task_planner_node          Sends prompt to local Ollama LLM via HTTP (libcurl).
    │                      Parses structured JSON → TaskCommand message.
    ▼
mission_manager_node       State machine: IDLE → NAVIGATE_TO_ROOM → PERCEIVE_OBJECT
    │                                           → PICK_OBJECT → NAVIGATE_TO_BASE → PLACE_OBJECT
    │
    ├──► navigation_manager_node   Wraps Nav2 NavigateToPose.
    │                              Manages named room waypoints (room1/2/3, base).
    │
    ├──► object_detector_node      Gazebo:    TF lookup of model pose (CPU, ground-truth).
    │                              Isaac Sim: grounding_dino + foundationpose pipeline.
    │                              Returns DetectedObject with 6-DoF pose.
    │
    └──► manipulation_manager_node Wraps MoveIt Task Constructor.
                                   Executes staged pick (7 steps) and place (6 steps).
                                   Manages planning scene collision objects.

slam_manager_node          Manages slam_toolbox lifecycle.
                           Services: start_mapping, save_map, start_localization.
```

---

## Package Structure

```
src/
├── amm_msgs/                   Custom ROS 2 interfaces
│   ├── msg/
│   │   ├── TaskCommand.msg         Parsed task: object, room, destination, action
│   │   └── DetectedObject.msg      Detected object with 6-DoF pose and confidence
│   └── action/
│       ├── ExecuteTask.action      Top-level: text prompt → success/fail
│       ├── PickObject.action       Pick a detected object
│       └── PlaceObject.action      Place at a target pose
│
├── amm_task_planner/           LLM interface node
│   └── task_planner_node       Receives text prompt via action server.
│                               Calls Ollama REST API (libcurl, no extra deps).
│                               Parses JSON response → TaskCommand.
│
├── amm_mission_manager/        High-level state machine
│   └── mission_manager_node    Drives the full pick-and-place pipeline.
│                               Action clients: Nav2, pick_object, place_object.
│
├── amm_navigation_manager/     Nav2 wrapper
│   └── navigation_manager_node Named waypoint store (room1/2/3, base).
│                               Sends NavigateToPose goals to Nav2.
│
├── amm_perception/             Object detection node (dual-backend)
│   └── object_detector_node    backend=gazebo:    TF lookup via ros_gz_bridge.
│                               backend=isaac_sim:  grounding_dino + foundationpose.
│                               Exposes detect(object_name) blocking call.
│
├── amm_manipulation/           MoveIt 2 / MTC wrapper
│   └── manipulation_manager_node  PickObject and PlaceObject action servers.
│                                  MTC staged pipelines with Cartesian approach/retreat.
│                                  Manages planning scene (collision objects).
│
├── amm_slam_manager/           SLAM lifecycle management
│   └── slam_manager_node       Services: start_mapping, save_map, start_localization.
│                               Controls slam_toolbox lifecycle node.
│
├── amm_gazebo/                 Gazebo Harmonic simulation assets
│   ├── worlds/
│   │   └── three_room.sdf          Three-room indoor world with pickable objects
│   ├── launch/
│   │   └── gazebo_sim.launch.py    Starts Gazebo, spawns robot, starts ros_gz_bridge
│   └── config/
│       └── ros_gz_bridge.yaml      Topic bridge config (Gazebo ↔ ROS 2)
│
├── amm_bringup/                Launch files and shared configuration
│   ├── launch/
│   │   └── simulation.launch.py    Full system bringup (one command, both simulators)
│   └── config/
│       ├── task_planner.yaml       Ollama host, model, timeout
│       ├── navigation.yaml         Room waypoints and base pose
│       ├── manipulation.yaml       MoveIt group names, motion parameters
│       ├── slam_toolbox.yaml       SLAM parameters
│       └── nav2_params.yaml        Nav2 full stack parameters (MPPI, holonomic)
│
└── amm_description/            Robot URDF / Xacro
    ├── urdf/
    │   └── ridgeback_franka.urdf.xacro   Ridgeback + FR3 combined model.
    │                                     Args: use_gazebo:=true adds Gazebo plugins
    │                                     (mecanum drive, gz_ros2_control, LiDAR,
    │                                      RGB-D camera, IMU).
    └── launch/
        └── description.launch.py         robot_state_publisher + joint_state_publisher
```

---

## Dependencies

### Install (apt)

```bash
# MoveIt Task Constructor + libcurl (for Ollama HTTP client)
sudo apt install \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-moveit-task-constructor-visualization \
  libcurl4-openssl-dev

# Already installed with ros-jazzy-desktop-full + Gazebo Harmonic:
#   ros-jazzy-navigation2, ros-jazzy-nav2-bringup, ros-jazzy-slam-toolbox,
#   ros-jazzy-moveit, ros-jazzy-ros2-control, ros-jazzy-ros2-controllers,
#   ros-jazzy-ros-gz-bridge, ros-jazzy-ros-gz-sim, ros-jazzy-gz-ros2-control,
#   ros-jazzy-mecanum-drive-controller, ros-jazzy-vision-msgs, ros-jazzy-tf2-ros
```

### Local LLM — Ollama (free, no account required)

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.2        # ~2 GB — runs on CPU or GPU
```

### Franka ROS 2 (source)

```bash
git clone -b jazzy https://github.com/frankaemika/franka_ros2.git src/franka_ros2
```

### Ridgeback description (source, if not available via apt)

```bash
git clone https://github.com/clearpathrobotics/ridgeback.git src/ridgeback
```

### Isaac ROS packages (GPU / Isaac Sim path only)

```
isaac_ros_visual_slam
isaac_ros_grounding_dino
isaac_ros_foundationpose
isaac_ros_cumotion
isaac_ros_cumotion_moveit
```

See the [Isaac ROS documentation](https://nvidia-isaac-ros.github.io) for installation.

---

## Build

```bash
cd ~/git/autonomous-mobile-manipulation-in-simulation
colcon build --symlink-install
source install/setup.bash
```

---

## Running the System

### Gazebo Harmonic (laptop / no GPU)

```bash
# Terminal 1 — start Ollama
ollama serve

# Terminal 2 — launch everything (Gazebo is the default)
ros2 launch amm_bringup simulation.launch.py

# Or explicitly:
ros2 launch amm_bringup simulation.launch.py simulator:=gazebo

# Headless (no GUI):
ros2 launch amm_bringup simulation.launch.py gz_gui:=false
```

### Isaac Sim (NVIDIA GPU required)

```bash
# 1. Open Isaac Sim, load the three-room scene, activate the ROS 2 bridge extension.
# 2. Start Ollama (Terminal 1)
ollama serve

# 3. Launch with Isaac Sim backend (Terminal 2)
ros2 launch amm_bringup simulation.launch.py simulator:=isaac_sim
```

### Send a task

```bash
ros2 action send_goal /execute_task amm_msgs/action/ExecuteTask \
  "{prompt: 'Pick up the apple in room 3 and bring it back to base'}"
```

The task planner parses the prompt with Llama 3.2, the mission manager drives the state machine, and the robot autonomously navigates, perceives, picks, and places the object.

---

## Data Flow Summary

```
"Pick up the apple in room 3 and bring it to base"
        │
        │  Ollama / Llama 3.2  (localhost:11434, free, CPU-capable)
        ▼
{ object_name: "apple", source_room: 3,
  destination: "base", action_type: "pick_and_place" }
        │
        │  Nav2 + slam_toolbox
        ▼
  Robot navigates to room 3
        │
        │  Gazebo:    TF lookup → ground-truth model pose  (CPU)
        │  Isaac Sim: grounding_dino + foundationpose       (GPU)
        ▼
  "apple" detected at pose (x, y, z, quat)
        │
        │  MoveIt Task Constructor (7-stage pick)
        ▼
  Franka FR3 grasps the apple
        │
        │  Nav2
        ▼
  Robot navigates to base
        │
        │  MoveIt Task Constructor (6-stage place)
        ▼
  Apple placed at base. Mission complete.
```

---

## License

MIT
