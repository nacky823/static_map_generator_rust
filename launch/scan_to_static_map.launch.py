# SPDX-FileCopyrightText: Yuki NAGAKI <youjiyongmu4@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
    )

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('static_map_generator_rust'),
        'config',
        'scan_to_static_map.rviz'
    ])

    scan_to_map_node = Node(
        package='scan_to_map_rust',
        executable='scan_to_map_rust',
        name='scan_to_map_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    static_map_generator_node = Node(
        package='static_map_generator_rust',
        executable='static_map_generator_rust',
        name='static_map_generator_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        prefix='xterm -e'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(scan_to_map_node)
    ld.add_action(static_map_generator_node)
    ld.add_action(rviz_node)

    return ld
