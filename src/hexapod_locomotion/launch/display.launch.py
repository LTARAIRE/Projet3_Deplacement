"""Launch only the URDF display in RViz (no locomotion node).

Useful for verifying the URDF model and joint slider testing.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_locomotion')
    desc_share = get_package_share_directory('phantomx_description')

    urdf_file = os.path.join(desc_share, 'urdf', 'phantomx.urdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'hexapod.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Pas de node de locomotion ici : on fournit une TF odom->base_link fixe
        # (le fixed frame RViz est "odom") à la hauteur de station debout.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0.088', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
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
