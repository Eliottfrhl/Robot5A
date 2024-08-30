import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build the MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("armr5", package_name="robot_moveit_config")
        .robot_description(file_path="config/armr5.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("robot_control"),
                "config",
                "moveit_control.yaml",
            )
        )
        .to_moveit_configs()
    )

    # MoveItCpp node
    moveit_cpp_node = Node(
        name="moveit_cpp_node",
        package="robot_control",  # Assuming your C++ executable is in the robot_control package
        executable="moveit_control",  # The name of your C++ executable
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([
        moveit_cpp_node,
    ])
