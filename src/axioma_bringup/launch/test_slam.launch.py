#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_axioma_description = get_package_share_directory('axioma_description')
    pkg_axioma_navigation = get_package_share_directory('axioma_navigation')

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_axioma_description, 'worlds', 'empty.world'),
            'verbose': 'true'
        }.items()
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'test_robot',
            '-file', os.path.join(pkg_axioma_description, 'models', 'simple_test_robot', 'model.sdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Robot state publisher (minimal TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': '<robot name="test_robot"><link name="base_link"/><link name="laser_link"/><joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser_link"/><origin xyz="0 0 0.1"/></joint></robot>'}
        ]
    )

    # SLAM Toolbox
    slam_params_file = os.path.join(pkg_axioma_navigation, 'config', 'slam_params.yaml')
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ]
    )

    # RViz
    rviz_config = os.path.join(pkg_axioma_description, 'rviz', 'slam-toolbox.yaml.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Teleop
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_robot,
        robot_state_publisher,
        slam_toolbox,
        rviz,
        teleop
    ])
