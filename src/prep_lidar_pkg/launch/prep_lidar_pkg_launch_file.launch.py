from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='prep_lidar_pkg',
            executable='prep_lidar',
            output='screen'),
    ])