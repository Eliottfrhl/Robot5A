import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    robot_pkg_name = 'robot_description'
    file_subpath = 'urdf/r5a_v_ros.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(robot_pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node for robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # Configure the RViz node
    rviz_config_file = os.path.join(get_package_share_directory(robot_pkg_name), 'config/default.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Configure the joint state publisher GUI node
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Run all nodes in the launch file
    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz,
        node_joint_state_publisher_gui

        
    ])


