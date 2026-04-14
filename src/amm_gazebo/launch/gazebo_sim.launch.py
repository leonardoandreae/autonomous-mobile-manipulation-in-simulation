"""
gazebo_sim.launch.py
====================
Starts Gazebo Harmonic with the three-room world and spawns the
Ridgeback + FR3 robot.  Also starts the ros_gz_bridge so that
Gazebo topics are accessible in the ROS 2 graph.

Usage (standalone):
    ros2 launch amm_gazebo gazebo_sim.launch.py

Normally called from amm_bringup/simulation.launch.py with
    simulator:=gazebo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ────────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    gui          = LaunchConfiguration("gz_gui",       default="true")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo clock (/clock topic)",
    )
    declare_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value="true",
        description="Launch the Gazebo GUI (set false for headless/CI)",
    )

    # ── Paths ─────────────────────────────────────────────────────────────────
    gazebo_share = get_package_share_directory("amm_gazebo")
    desc_share   = get_package_share_directory("amm_description")

    # ── Gazebo resource path — lets Gazebo resolve model://amm_description/... ─
    # URDF→SDF conversion rewrites package:// URIs to model://<pkg>/<path>.
    # Gazebo searches GZ_SIM_RESOURCE_PATH for a directory named <pkg>.
    # get_package_share_directory returns .../share/amm_description;
    # its parent (.../share) contains the amm_description/ subfolder Gazebo needs.
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(desc_share)
              + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    world_file = os.path.join(gazebo_share, "worlds", "three_room.sdf")
    bridge_cfg = os.path.join(gazebo_share, "config", "ros_gz_bridge.yaml")
    urdf_xacro = os.path.join(desc_share,   "urdf",   "ridgeback_franka.urdf.xacro")

    # ── Process xacro → URDF string at launch time ───────────────────────────
    # xacro cannot be parsed by Gazebo directly; we pre-process it here so that
    # robot_state_publisher gets a plain URDF string, and the spawner reads
    # it from the /robot_description topic.
    robot_description = ParameterValue(
        Command(["xacro ", urdf_xacro, " use_gazebo:=true"]),
        value_type=str,
    )

    # ── Gazebo Harmonic ───────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
            "on_exit_shutdown": "True",
        }.items(),
    )

    # ── robot_state_publisher — publishes processed URDF to /robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time":       use_sim_time,
            "robot_description":  robot_description,
        }],
    )

    # ── Spawn robot into Gazebo from the /robot_description topic ─────────────
    # ros_gz_sim's 'create' executable reads the URDF from the ROS topic so
    # Gazebo never has to parse xacro syntax.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_ridgeback_franka",
        output="screen",
        arguments=[
            "-world", "three_room_world",
            "-topic", "/robot_description",
            "-name",  "ridgeback_franka",
            "-x",     "0.0",
            "-y",     "0.0",
            "-z",     "0.05",
            "-Y",     "0.0",
        ],
    )

    # ── ros_gz_bridge ──────────────────────────────────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "config_file":  bridge_cfg,
            "qos_overrides./tf.publisher.durability": "transient_local",
        }],
    )

    # ── ros2_control controller spawners ──────────────────────────────────────
    # Spawned after the robot is created in Gazebo so that gz_ros2_control
    # has already loaded the hardware interface.
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_fr3_arm_controller",
        arguments=["fr3_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_fr3_gripper_controller",
        arguments=["fr3_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        gz_resource_path,
        declare_use_sim_time,
        declare_gui,
        LogInfo(msg="=== AMM Gazebo Harmonic Simulation ==="),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        spawn_jsb,
        spawn_arm,
        spawn_gripper,
    ])
