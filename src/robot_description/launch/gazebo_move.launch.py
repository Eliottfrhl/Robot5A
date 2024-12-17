"""
@file gazebo_move.launch.py
@brief Launch file for setting up the robot simulation and MoveIt configurations.

This launch file initializes the robot simulation in Gazebo, spawns the robot entity,
sets up the MoveIt configuration, and starts the necessary nodes and controllers
for the robot operation.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    @brief Generates the launch description for the robot simulation and MoveIt setup.
    This function sets up the robot description, launches Gazebo, spawns the robot entity,
    configures MoveIt, and starts the necessary nodes and controllers.

    @return LaunchDescription object containing all the nodes and configurations to launch.
    """

    # Specify the name of the package and path to xacro file within the package
    pkg_name = "robot_description"  # Name of the robot description package
    share_dir = get_package_share_directory(
        pkg_name
    )  # Get the share directory of the package

    # Use xacro to process the file
    xacro_file = os.path.join(
        share_dir, "urdf", "r5a_v_ros.urdf.xacro"
    )  # Full path to the XACRO file
    robot_description_xacro = xacro.process_file(xacro_file)  # Process the XACRO file
    robot_urdf = (
        robot_description_xacro.toxml()
    )  # Convert the processed XACRO to URDF XML

    # Configure the robot_state_publisher node
    node_robot_state_publisher = Node(
        package="robot_state_publisher",  # Package containing the node
        executable="robot_state_publisher",  # Executable name
        output="screen",  # Output mode
        parameters=[
            {"robot_description": robot_urdf},
            {"use_sim_time": True},
        ],  # Parameters
    )

    # Node to spawn the entity in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",  # Package containing the node
        executable="spawn_entity.py",  # Executable script to spawn entities
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "armr5",
        ],  # Arguments for spawning
        output="screen",
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
    )

    # Commands to load and start controllers after spawning the robot
    load_joint_states_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],  # Command to load and activate joint_state_broadcaster
        output="screen",
    )

    load_arm_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "arm_controller",
        ],  # Command to load and activate arm_controller
        output="screen",
    )

    # MoveIt configuration using MoveItConfigsBuilder
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
    use_sim_time = {"use_sim_time": True}
    config_dict.update(use_sim_time)

    # Launch the Move Group node
    move_group_node = Node(
        package="moveit_ros_move_group",  # Package containing the move_group node
        executable="move_group",  # Executable name
        output="screen",
        parameters=[config_dict],  # Parameters including MoveIt configurations
    )

    # Return the LaunchDescription with all the nodes and event handlers
    return LaunchDescription(
        [
            gazebo,
            node_robot_state_publisher,
            spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_states_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_states_controller,
                    on_exit=[load_arm_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_arm_controller,
                    on_exit=[move_group_node],
                )
            ),
        ]
    )
