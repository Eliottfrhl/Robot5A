"""
@file moveit_control2.launch.py
@brief Launch file for the robot control node.

This launch file initializes the MoveIt configurations and starts the robot control node
that interfaces with the MoveIt planning framework.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    @brief Generates the launch description for the robot control node.

    This function loads the MoveIt configurations and initializes the robot control node.

    @return LaunchDescription object containing the robot control node.
    """

    # Load the MoveIt configuration using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("robot_moveit_config", package_name="robot_moveit_config")
        .robot_description(
            file_path="config/r5a_v_ros.urdf.xacro", mappings={"use_sim_time": "true"}
        )
        .robot_description_semantic("config/armr5.srdf")
        .robot_description_kinematics("config/kinematics.yaml")
        .joint_limits("config/joint_limits.yaml")
        .trajectory_execution("config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    config_dict = moveit_config.to_dict()

    # Launch your C++ node that controls the robot
    control_node = Node(
        package="robot_control",  # Replace with your package name
        executable="moveit_control_simple",  # Replace with your executable name
        output="screen",
        parameters=[config_dict],
    )

    return LaunchDescription([control_node])
