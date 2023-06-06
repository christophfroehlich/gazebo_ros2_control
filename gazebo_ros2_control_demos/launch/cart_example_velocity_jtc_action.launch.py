import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch stuff using ROS 2."""

    return LaunchDescription([
        Node(
            package='gazebo_ros2_control_demos',
            executable='example_position_with_velocity',
            name='example_position_with_velocity',
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ])