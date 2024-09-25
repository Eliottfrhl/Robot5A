## Project Description

This project involves the simulation and control of a robotic arm using ROS2 Humble, Gazebo, and MoveIt2. The robotic arm is defined in URDF and is capable of executing planned trajectories within a simulated environment. The project is structured to support the integration of ROS2 control and MoveIt2 for motion planning and trajectory execution.

## Prerequisites

- Operating System: Ubuntu 22.04
- ROS2 Distribution: ROS2 Humble
- Simulation Environment: Gazebo
- Motion Planning: MoveIt2

## Requirements Instalation

```bash
sudo apt update && sudo apt install -y \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-joint-state-broadcaster \
ros-humble-robot-state-publisher \
ros-humble-controller-manager \
ros-humble-joint-trajectory-controller \
ros-humble-rqt \
ros-humble-rqt-common-plugins \
ros-humble-rqt-graph \
ros-humble-gazebo-ros-pkgs \
ros-humble-gazebo-ros2-control \
ros-humble-xacro \
ros-humble-diagnostic-updater \
ros-humble-rviz2 \
ros-humble-tf2-tools \
ros-humble-ros2bag
meshlab \
```

## Usage

Source the ROS2 and project setup files:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the Gazebo simulation with the robotic arm:

```bash
ros2 launch robot_description gazebo.launch.py
```

Move the arm using RViz

```bash
ros2 launch robot_moveit_config moveit_gazebo.launch.py
```

Move the arm using a C++ Node

```bash
ros2 launch robot_control moveit_control2.launch.py
```

## Project Architecture

### robot_control

This package will contain the C++ nodes for controlling the robotic arm, integrating with ROS2 control and MoveIt2.


### robot_description

This package contains the URDF and associated files for defining the robotic arm in simulation.

- config/controllers.yaml: Configuration for the controllers used in ROS2 control.
- launch: Contains the launch files for bringing up the Gazebo simulation (gazebo.launch.py is the main one).
- meshes: 3D models of the robot's components.
- urdf: URDF and xacro files that define the robotic arm.
    * r5a_v_ros.urdf.xacro: Main URDF file for the robot.
    * The rest is another way to present the urdf

### robot_moveit_config

This package configures MoveIt2 for the robotic arm, including motion planning and control settings.

- config: Contains various configuration files for MoveIt2.
    * armr5.ros2_control.xacro: Xacro file specific to ROS2 control for the arm.
    * armr5.srdf: Semantic Robot Description Format file for MoveIt2.
    * initial_positions.yaml: Initial joint positions for the robot.
    * joint_limits.yaml: Defines joint limits and constraints.
    * kinematics.yaml: Kinematics configurations for the robot.
    * moveit_controllers.yaml: Configuration for MoveIt2 controllers.
    * moveit.rviz: RViz configuration for visualizing the robot.
    * pilz_cartesian_limits.yaml: Cartesian limits for specific motion planning.
    * ros2_controllers.yaml: ROS2 control configurations.
    * sensors_3d.yaml: Configuration for 3D sensors (if applicable).
- launch: Launch files for MoveIt2 and related nodes.
    * demo.launch.py: A demo launch file for MoveIt2.
    * move_group.launch.py: Launch file for the MoveIt2 move group.
    * moveit_gazebo.launch.py: Launch file integrating MoveIt2 with Gazebo.
    * moveit_rviz.launch.py: Launch file to bring up RViz with MoveIt2.
    * rsp.launch.py: Robot State Publisher launch file.
    * setup_assistant.launch.py: Launch file for the MoveIt2 setup assistant.
    * spawn_controllers.launch.py: Launch file to spawn the controllers.
    * static_virtual_joint_tfs.launch.py: Launch file for static TFs.
    * warehouse_db.launch.py: Launch file for the warehouse database.

- .setup_assistant: setup assistant configuration for this project in particular.
