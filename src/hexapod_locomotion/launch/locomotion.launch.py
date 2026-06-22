"""Launch the hexapod locomotion system with RViz visualization."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_locomotion')
    desc_share = get_package_share_directory('phantomx_description')

    urdf_file = os.path.join(desc_share, 'urdf', 'phantomx.urdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'hexapod.rviz')
    params_file = os.path.join(pkg_share, 'config', 'hexapod_params.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2'
        ),

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
            executable='world_node',
            name='hexapod_world',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
        ),
    ])
