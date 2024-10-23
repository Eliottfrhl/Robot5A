"""
@file gazebo_gui.launch.py
@brief Launch file for setting up the robot simulation and MoveIt configurations.

This launch file initializes the robot simulation in Gazebo, spawns the robot entity,
sets up the MoveIt configuration, and starts the necessary nodes and controllers
for the robot operation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    @brief Generates the launch description for the robot simulation and MoveIt setup.

    This function sets up the robot description, launches Gazebo, spawns the robot entity,
    configures MoveIt, and starts the necessary nodes and controllers.

    @return LaunchDescription object containing all the nodes and configurations to launch.
    """

    # Package Directories
    pkg_name = "robot_description"  # Name of the robot description package
    robot_moveit_config = (
        "robot_moveit_config"  # Name of the MoveIt configuration package
    )
    share_dir = get_package_share_directory(
        pkg_name
    )  # Path to the robot description package
    moveit_config_pkg_path = get_package_share_directory(
        robot_moveit_config
    )  # Path to the MoveIt config package

    # Load and process URDF/XACRO file
    xacro_file = os.path.join(
        share_dir, "urdf", "r5a_v_ros.urdf.xacro"
    )  # Path to the XACRO file
    robot_description_config = xacro.process_file(xacro_file)  # Process the XACRO file
    robot_description = {
        "robot_description": robot_description_config.toxml()
    }  # Convert to XML format

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Spawn Entity Node
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "armr5"],
        output="screen",
    )

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_moveit_config, package_name=robot_moveit_config)
        .robot_description(file_path=xacro_file, mappings={"use_sim_time": "true"})
        .robot_description_semantic(
            os.path.join(moveit_config_pkg_path, "config", "armr5.srdf")
        )
        .robot_description_kinematics(
            os.path.join(moveit_config_pkg_path, "config", "kinematics.yaml")
        )
        .trajectory_execution(
            os.path.join(moveit_config_pkg_path, "config", "moveit_controllers.yaml")
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    # Load Controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
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
        ],
        output="screen",
    )

    # Launch the GUI Node
    gui_node = Node(
        package="robot_control",  # Package name containing the GUI node
        executable="moveit_control_gui",  # Executable name of the GUI node
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"moveit_current_state_monitor.joint_state_qos": "sensor_data"},
        ],  # Pass the MoveIt config to the GUI node
        # arguments=['--ros-args', '--log-level', 'debug'],
    )

    # Return the LaunchDescription
    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),  # Enable simulation time
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_arm_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_arm_controller,
                    on_exit=[move_group_node],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=move_group_node,
                    on_start=[
                        gui_node,
                    ],
                )
            ),
            gazebo,
            robot_state_publisher_node,
            spawn_entity,
        ]
    )
