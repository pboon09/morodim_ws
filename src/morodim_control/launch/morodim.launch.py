#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Joy driver node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    
    # Custom joy control node
    joy_control_node = Node(
        package='morodim_control',
        executable='morodim_joy.py',
        name='morodim_joy',
        parameters=[{
            'max_linear_speed': 0.5,
            'max_angular_speed': 0.8,
            'joint_step': 0.1
        }],
        output='screen'
    )
    
    return LaunchDescription([
        joy_node,
        joy_control_node
    ])