"""
@file gazebo.launch.py
@brief Launch file for setting up the robot simulation in Gazebo.

This launch file initializes the robot simulation in Gazebo, spawns the robot entity,
and starts the necessary nodes and controllers for the robot operation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """
    @brief Generates the launch description for the robot simulation.

    This function sets up the robot description, launches Gazebo, spawns the robot entity,
    and starts the necessary nodes and controllers.

    @return LaunchDescription object containing all the nodes and configurations to launch.
    """

    # Specify the name of the package and path to xacro file within the package
    pkg_name = "robot_description"  # Name of the robot description package
    file_subpath = (
        "urdf/r5a_v_ros.urdf.xacro"  # Path to the XACRO file within the package
    )

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

    # Joint State Publisher Node
    node_joint_state_publisher = Node(
        name="joint_state_publisher",  # Name of the node
        package="joint_state_publisher",  # Package containing the node
        executable="joint_state_publisher",  # Executable name
        output="screen",
    )

    # Controller manager node
    ros2_control_node = Node(
        package="controller_manager",  # Package containing the node
        executable="ros2_control_node",  # Executable name
        parameters=[
            os.path.join(
                get_package_share_directory(pkg_name), "config", "controllers.yaml"
            ),
        ],  # Parameters including path to controllers configuration
        output="screen",
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

    # Return the LaunchDescription with all the nodes and event handlers
    return LaunchDescription(
        [
            gazebo,
            node_robot_state_publisher,
            # node_joint_state_publisher,  # Uncomment if joint_state_publisher is needed
            spawn_entity,
            # ros2_control_node,  # Uncomment if ros2_control_node is needed
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
        ]
    )
