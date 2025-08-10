from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tesikaga_lidar_detector',
            executable='run_calibration',
            name='calibration_client_node',
            output='screen'
        )
    ])
