"""
simulation.launch.py
====================
Full system launch for Isaac Sim simulation.

Usage:
    ros2 launch amm_bringup simulation.launch.py

Pre-requisites:
    1. Isaac Sim is running with the three-room scene loaded.
    2. Ollama is running: `ollama serve` (and model pulled: `ollama pull llama3.2`)
    3. The ROS 2 bridge extension is active inside Isaac Sim.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ────────────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    log_level    = LaunchConfiguration("log_level",    default="info")

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Isaac Sim clock (/clock topic)"
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error"],
        description="ROS 2 log level"
    )

    # ── Config paths ─────────────────────────────────────────────────────────────
    bringup_share       = FindPackageShare("amm_bringup")
    nav2_share          = FindPackageShare("nav2_bringup")
    slam_toolbox_share  = FindPackageShare("slam_toolbox")

    task_planner_cfg    = PathJoinSubstitution([bringup_share, "config", "task_planner.yaml"])
    nav_manager_cfg     = PathJoinSubstitution([bringup_share, "config", "navigation.yaml"])
    nav2_params         = PathJoinSubstitution([bringup_share, "config", "nav2_params.yaml"])
    slam_cfg            = PathJoinSubstitution([bringup_share, "config", "slam_toolbox.yaml"])
    manipulation_cfg    = PathJoinSubstitution([bringup_share, "config", "manipulation.yaml"])

    # ── SLAM ─────────────────────────────────────────────────────────────────────
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_toolbox_share, "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
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

    # ── AMM nodes ────────────────────────────────────────────────────────────────
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

    mission_executive_node = Node(
        package="amm_mission_executive",
        executable="mission_executive_node",
        name="mission_executive_node",
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

    object_detector_node = Node(
        package="amm_perception",
        executable="object_detector_node",
        name="object_detector_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    manipulation_manager_node = Node(
        package="amm_manipulation",
        executable="manipulation_manager_node",
        name="manipulation_manager_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[manipulation_cfg, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_sim_time,
        declare_log_level,
        LogInfo(msg="=== Autonomous Mobile Manipulation — Simulation Bringup ==="),
        slam_toolbox_node,
        nav2_bringup,
        slam_manager_node,
        navigation_manager_node,
        task_planner_node,
        mission_executive_node,
        object_detector_node,
        manipulation_manager_node,
    ])
