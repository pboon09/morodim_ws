#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    
    # Get the package directory
    package_name = 'morodim_description'  # Replace with your actual package name
    pkg_share = get_package_share_directory(package_name)
    
    # RViz configuration path
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'gazebo.rviz')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'morodim.urdf.xacro')
    
    # Use xacro to process the file
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' tf_prefix:=""'
    ])
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # Joint state publisher GUI node for controlling joints
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Create default RViz configuration if it doesn't exist
    rviz_config_dir = os.path.join(pkg_share, 'config')
    if not os.path.exists(rviz_config_dir):
        os.makedirs(rviz_config_dir)
    
    if not os.path.exists(rviz_config_path):
        # Create a basic RViz configuration file
        create_rviz_config = ExecuteProcess(
            cmd=['touch', rviz_config_path],
            output='screen'
        )
    else:
        create_rviz_config = None
    
    # Create and return launch description
    launch_description = LaunchDescription()
    
    # Add all nodes to the launch description
    if create_rviz_config:
        launch_description.add_action(create_rviz_config)
    
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher_gui)
    launch_description.add_action(rviz)
    
    return launch_description