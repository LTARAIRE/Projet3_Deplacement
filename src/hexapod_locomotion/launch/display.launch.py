"""Launch only the URDF display in RViz (no locomotion node).

Useful for verifying the URDF model and joint slider testing.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_locomotion')

    urdf_file = os.path.join(pkg_share, 'urdf', 'hexapod.urdf.xacro')
    rviz_file = os.path.join(pkg_share, 'rviz', 'hexapod.rviz')

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
        ),
    ])
