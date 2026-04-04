# Autonomous Mobile Manipulation in Simulation

A ROS 2 system where a mobile manipulator autonomously interprets natural language task prompts and executes pick-and-place operations inside a simulated three-room environment. The robot navigates between rooms, identifies objects using open-vocabulary vision, grasps them with a robot arm, and returns them to a base station — all driven from a single text instruction.

**Maintainer:** Leonardo Andreae — leoandreae@gmail.com

---

## Robot Platform

**Clearpath Ridgeback + Franka FR3**

| Component | Details |
|---|---|
| Mobile base | Clearpath Ridgeback — omnidirectional (holonomic), 4-wheel mecanum drive |
| Arm | Franka FR3 — 7 DoF, torque-controlled |
| Gripper | Franka Hand — parallel gripper with force/torque sensing |
| Sensors | 2× LiDAR (Ridgeback), wrist RGB-D camera, IMU |
| Compute | NVIDIA Jetson / workstation with GPU (for Isaac ROS inference) |

This combination is the recommended platform because `ridgeback_franka.usd` ships as a **native asset inside Isaac Sim** — no URDF conversion or physics tuning required. The holonomic base allows precise positioning relative to objects without multi-step turning manoeuvres.

---

## Simulator

**NVIDIA Isaac Sim** (Omniverse) — free for non-commercial and research use.

The simulation scene contains:
- Three labelled rooms, each containing placeable objects (e.g. apple, bottle, cube)
- A base area where the robot deposits retrieved objects
- Full physics simulation of the robot, objects, and environment
- ROS 2 bridge publishing sensor data (LiDAR, RGB-D, joint states, odometry)

---

## Middleware

| Layer | Technology |
|---|---|
| Middleware | ROS 2 Jazzy Jalisco (C++) |
| Build system | colcon / CMake |
| SLAM | slam_toolbox (2D LiDAR) + isaac_ros_visual_slam (visual-inertial) |
| Navigation | Nav2 (MPPI controller, Navfn planner, layered costmaps) |
| Manipulation | MoveIt 2 + MoveIt Task Constructor (MTC) |
| GPU motion planning | isaac_ros_cumotion (CUDA-accelerated collision-free trajectories) |
| Object detection | isaac_ros_grounding_dino (open-vocabulary 2D detection) |
| Pose estimation | isaac_ros_foundationpose (6-DoF object pose) |
| LLM | Ollama — local, free, no API key (models: Llama 3.2 / Mistral) |

---

## System Architecture

Natural language flows through a chain of ROS 2 action servers:

```
User prompt
    │
    ▼
task_planner_node          Sends prompt to local Ollama LLM via HTTP.
    │                      Parses structured JSON → publishes TaskCommand.
    ▼
mission_executive_node     State machine: IDLE → NAVIGATE → PERCEIVE
    │                                           → PICK → RETURN → PLACE
    ├──► navigation_manager_node   Wraps Nav2 NavigateToPose.
    │                              Manages named room waypoints.
    │
    ├──► object_detector_node      Bridges isaac_ros_grounding_dino
    │                              and isaac_ros_foundationpose.
    │                              Returns DetectedObject with 6-DoF pose.
    │
    └──► manipulation_manager_node Wraps MoveIt Task Constructor.
                                   Executes staged pick (7 steps)
                                   and place (6 steps) pipelines.

slam_manager_node          Manages slam_toolbox lifecycle.
                           Saves/loads maps, switches mapping ↔ localisation.
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
│                               Calls Ollama REST API (libcurl).
│                               Parses JSON response → TaskCommand.
│
├── amm_mission_executive/      High-level state machine
│   └── mission_executive_node  Drives the full pick-and-place pipeline.
│                               Action clients for Nav2, perception, manipulation.
│
├── amm_navigation_manager/     Nav2 wrapper
│   └── navigation_manager_node Named waypoint store (room1/2/3, base).
│                               Sends NavigateToPose goals to Nav2.
│
├── amm_perception/             Vision pipeline wrapper
│   └── object_detector_node    Subscribes to isaac_ros_grounding_dino detections.
│                               Subscribes to isaac_ros_foundationpose poses.
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
├── amm_bringup/                Launch files and shared configuration
│   ├── launch/
│   │   └── simulation.launch.py    Full system bringup (one command)
│   └── config/
│       ├── task_planner.yaml       Ollama host, model, timeout
│       ├── navigation.yaml         Room waypoints and base pose
│       ├── manipulation.yaml       MoveIt group names, motion parameters
│       ├── slam_toolbox.yaml       SLAM parameters
│       └── nav2_params.yaml        Nav2 full stack parameters
│
└── amm_description/            Robot URDF / Xacro
    ├── urdf/
    │   └── ridgeback_franka.urdf.xacro   Ridgeback + FR3 combined model
    └── launch/
        └── description.launch.py         robot_state_publisher + joint_state_publisher
```

---

## Dependencies

### ROS 2 packages (apt)
```bash
sudo apt install \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-moveit \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ridgeback-description \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-msgs \
  ros-jazzy-pcl-ros \
  libcurl4-openssl-dev
```

### Isaac ROS packages (source, from NVIDIA)
```
isaac_ros_visual_slam
isaac_ros_nvblox
isaac_ros_grounding_dino
isaac_ros_foundationpose
isaac_ros_cumotion
isaac_ros_cumotion_moveit
```
See the [Isaac ROS documentation](https://nvidia-isaac-ros.github.io) for installation instructions.

### Franka ROS 2 (source)
```bash
# franka_ros2 — Jazzy branch
git clone -b jazzy https://github.com/frankaemika/franka_ros2.git
```

### Local LLM — Ollama (free, no account required)
```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.2        # ~2 GB — runs on CPU or GPU
```

---

## Build

```bash
cd ~/git/autonomous-mobile-manipulation-in-simulation
colcon build --symlink-install
source install/setup.bash
```

---

## Running the System

### 1. Start Isaac Sim
Open Isaac Sim, load the three-room scene, and activate the ROS 2 bridge extension.

### 2. Start Ollama
```bash
ollama serve
```

### 3. Launch everything
```bash
ros2 launch amm_bringup simulation.launch.py
```

### 4. Send a task
```bash
ros2 action send_goal /execute_task amm_msgs/action/ExecuteTask \
  "{prompt: 'Pick up the apple in room 3 and bring it back to base'}"
```

The task planner parses the prompt with the local LLM, the mission executive drives the state machine, and the robot autonomously navigates, perceives, picks, and places the object.

---

## Data Flow Summary

```
"Pick up the apple in room 3 and bring it to base"
        │
        │  Ollama (Llama 3.2, local HTTP)
        ▼
{ object_name: "apple", source_room: 3,
  destination: "base", action_type: "pick_and_place" }
        │
        │  Nav2 + slam_toolbox
        ▼
  Robot navigates to room 3
        │
        │  isaac_ros_grounding_dino + isaac_ros_foundationpose
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
