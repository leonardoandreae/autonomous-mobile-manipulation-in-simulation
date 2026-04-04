"""
description.launch.py
======================
Publishes the robot URDF to /robot_description and starts robot_state_publisher.
Useful for RViz2 visualisation and TF tree without running the full simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    xacro_file = PathJoinSubstitution([
        FindPackageShare("amm_description"),
        "urdf",
        "ridgeback_franka.urdf.xacro",
    ])

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        robot_state_publisher,
        joint_state_publisher,
    ])
