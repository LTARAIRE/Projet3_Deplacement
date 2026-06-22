"""Launch locomotion + keyboard teleop for manual control."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_locomotion')

    urdf_file = os.path.join(pkg_share, 'urdf', 'hexapod.urdf.xacro')
    rviz_file = os.path.join(pkg_share, 'rviz', 'hexapod.rviz')
    params_file = os.path.join(pkg_share, 'config', 'hexapod_params.yaml')

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        Node(
            package='hexapod_locomotion',
            executable='locomotion_node',
            name='hexapod_locomotion',
            parameters=[params_file],
            output='screen',
        ),

        Node(
            package='hexapod_locomotion',
            executable='teleop_key_node',
            name='hexapod_teleop_key',
            output='screen',
            prefix='xterm -e',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
        ),
    ])
