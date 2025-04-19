# Morodim
This package provides a ROS2 implementation for a mobile manipulator robot that combines a MiR100 mobile base with a 3-DOF manipulator arm. It includes URDF models, controllers, launch files, and a joystick interface for controlling both the mobile base and the robotic arm.

## Features
- Complete URDF model integrating MiR100 mobile base with a custom 3-DOF manipulator
- ROS2 controllers for both differential drive (mobile base) and position control (manipulator)
- Joystick control interface for simultaneous operation of the mobile base and manipulator
- Gazebo simulation support for testing and development
- RViz visualization configurations

## Dependencies
- ROS2 (tested on Humble)
- Gazebo
- joy package (ros-humble-joy)
- controller_manager
- MiR description package
- ros2_control and gazebo_ros2_control

## Installation
### Installing ROS2 Dependencies
```bash
sudo apt-get update
sudo apt-get install ros-humble-joy ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-diff-drive-controller ros-humble-joint-trajectory-controller
```
### Building the Package
```bash
# Clone the repository
git clone https://github.com/pboon09/morodim_ws.git

cd ~/morodim_ws
colcon build --symlink-install

source ~/morodim_ws/install/setup.bash
```

## Usage
### Launching the Robot in Simulation
```bash
ros2 launch morodim_description simple.launch.py
```
This launch file:
- Starts Gazebo simulator
- Loads the robot URDF using robot_state_publisher
- Spawns the robot in Gazebo
- Launches RViz for visualization
- Starts the joint_state_broadcaster and joint_trajectory_position_controller
### Joystick Control
```bash
ros2 launch morodim_control joy_control.launch.py
```
This launch file:
- Starts the joy_node to capture joystick input
- Runs the morodim_joy.py node for custom joystick-to-robot control
- Sets parameters for control sensitivity and speed
## Joystick Controls
- Left analog stick: Controls the mobile base
  - Forward/backward: Linear velocity
  - Left/right: Angular velocity (rotation)
- L2/R2 triggers: Control joint 1 (base rotation)
  - L2: Rotate counterclockwise
  - R2: Rotate clockwise
- Triangle/X buttons: Control joint 2 (shoulder)
  - Triangle: Positive rotation
  - X: Negative rotation
- L1/R1 bumpers: Control joint 3 (elbow)
  - L1: Negative rotation
  - R1: Positive rotation

## Feedback
If you have any feedback or issues, please create an issue on the GitHub repository.
