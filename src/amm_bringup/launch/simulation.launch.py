"""
simulation.launch.py
====================
Full system launch for the AMM simulation.

Supports two simulators selected via the `simulator` argument:

    simulator:=gazebo      (default) — Gazebo Harmonic, CPU-only, no GPU needed
    simulator:=isaac_sim   — NVIDIA Isaac Sim, requires NVIDIA GPU + CUDA 12.x

Usage:
    # Gazebo (laptop, integrated graphics)
    ros2 launch amm_bringup simulation.launch.py

    # Isaac Sim (workstation / cloud with NVIDIA GPU)
    ros2 launch amm_bringup simulation.launch.py simulator:=isaac_sim

Pre-requisites (Gazebo):
    1. Ollama is running: `ollama serve`  (model: `ollama pull llama3.2`)
    2. All packages built: `colcon build --symlink-install`

Pre-requisites (Isaac Sim):
    1. Isaac Sim is running with the three-room scene loaded.
    2. The ROS 2 bridge extension is active inside Isaac Sim.
    3. Ollama is running.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ────────────────────────────────────────────────────────────────
    simulator    = LaunchConfiguration("simulator",    default="gazebo")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    log_level    = LaunchConfiguration("log_level",    default="info")
    gz_gui       = LaunchConfiguration("gz_gui",       default="true")

    declare_simulator = DeclareLaunchArgument(
        "simulator",
        default_value="gazebo",
        choices=["gazebo", "isaac_sim"],
        description="Simulator backend: 'gazebo' (CPU, Gazebo Harmonic) or "
                    "'isaac_sim' (GPU, NVIDIA Isaac Sim)",
    )
    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulator clock (/clock topic)",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error"],
        description="ROS 2 log level",
    )
    declare_gz_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value="true",
        description="Show Gazebo GUI (only used when simulator:=gazebo)",
    )

    # ── Convenience conditions ────────────────────────────────────────────────────
    is_gazebo     = PythonExpression(["'", simulator, "' == 'gazebo'"])
    is_isaac_sim  = PythonExpression(["'", simulator, "' == 'isaac_sim'"])

    # ── Config paths ─────────────────────────────────────────────────────────────
    bringup_share      = FindPackageShare("amm_bringup")
    nav2_share         = FindPackageShare("nav2_bringup")
    slam_toolbox_share = FindPackageShare("slam_toolbox")
    gazebo_share       = FindPackageShare("amm_gazebo")

    task_planner_cfg  = PathJoinSubstitution([bringup_share, "config", "task_planner.yaml"])
    nav_manager_cfg   = PathJoinSubstitution([bringup_share, "config", "navigation.yaml"])
    nav2_params       = PathJoinSubstitution([bringup_share, "config", "nav2_params.yaml"])
    slam_cfg          = PathJoinSubstitution([bringup_share, "config", "slam_toolbox.yaml"])
    manipulation_cfg  = PathJoinSubstitution([bringup_share, "config", "manipulation.yaml"])

    # ── Simulator-specific: Gazebo ───────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_share, "launch", "gazebo_sim.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gz_gui":       gz_gui,
        }.items(),
        condition=IfCondition(is_gazebo),
    )

    # ── SLAM ─────────────────────────────────────────────────────────────────────
    # slam_toolbox: CPU-based 2D SLAM — used with both simulators
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_toolbox_share, "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "use_sim_time":    use_sim_time,
            "slam_params_file": slam_cfg,
        }.items(),
    )

    # ── Nav2 ─────────────────────────────────────────────────────────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file":  nav2_params,
        }.items(),
    )

    # ── AMM nodes (simulator-agnostic) ───────────────────────────────────────────
    slam_manager_node = Node(
        package="amm_slam_manager",
        executable="slam_manager_node",
        name="slam_manager_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    task_planner_node = Node(
        package="amm_task_planner",
        executable="task_planner_node",
        name="task_planner_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[task_planner_cfg, {"use_sim_time": use_sim_time}],
    )

    mission_manager_node = Node(
        package="amm_mission_manager",
        executable="mission_manager_node",
        name="mission_manager_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[nav_manager_cfg, {"use_sim_time": use_sim_time}],
    )

    navigation_manager_node = Node(
        package="amm_navigation_manager",
        executable="navigation_manager_node",
        name="navigation_manager_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[nav_manager_cfg, {"use_sim_time": use_sim_time}],
    )

    manipulation_manager_node = Node(
        package="amm_manipulation",
        executable="manipulation_manager_node",
        name="manipulation_manager_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[manipulation_cfg, {"use_sim_time": use_sim_time}],
    )

    # ── Perception: Gazebo variant ────────────────────────────────────────────────
    # Reads object poses directly from Gazebo model states — no GPU vision pipeline
    object_detector_gz = Node(
        package="amm_perception",
        executable="object_detector_node",
        name="object_detector_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "use_sim_time": use_sim_time,
            "backend": "gazebo",
            "gz_world_name": "three_room_world",
        }],
        condition=IfCondition(is_gazebo),
    )

    # ── Perception: Isaac Sim variant ─────────────────────────────────────────────
    # Uses isaac_ros_grounding_dino + isaac_ros_foundationpose pipeline
    object_detector_isaac = Node(
        package="amm_perception",
        executable="object_detector_node",
        name="object_detector_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "use_sim_time": use_sim_time,
            "backend": "isaac_sim",
            "rgb_topic":         "/rgb/image_raw",
            "detections_topic":  "/detections",
            "pose_topic":        "/object_poses",
        }],
        condition=IfCondition(is_isaac_sim),
    )

    return LaunchDescription([
        declare_simulator,
        declare_sim_time,
        declare_log_level,
        declare_gz_gui,
        LogInfo(msg=["=== AMM Simulation Bringup | simulator=", simulator, " ==="]),

        # Simulator backend
        gazebo_launch,

        # Core ROS 2 stack
        slam_toolbox_node,
        nav2_bringup,

        # AMM nodes
        slam_manager_node,
        navigation_manager_node,
        task_planner_node,
        mission_manager_node,
        manipulation_manager_node,

        # Perception (one of these runs depending on simulator)
        object_detector_gz,
        object_detector_isaac,
    ])
