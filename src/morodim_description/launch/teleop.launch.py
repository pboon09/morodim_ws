#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare launch argument for cmd_vel_topic
    declare_cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',  # MiR uses this topic by default
        description='Topic for velocity commands'
    )
    
    # Teleop node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # This opens in a separate terminal window
        remappings=[
            ('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        declare_cmd_vel_topic_arg,
        teleop_node
    ])