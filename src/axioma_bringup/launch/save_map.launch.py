#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_saver',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-f', '/home/axioma/ros2/axioma_humble_ws/src/axioma_navigation/maps/mapa']
        )
    ])

