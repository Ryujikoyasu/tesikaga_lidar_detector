# launch/calibration.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    detector_config = os.path.join(
        get_package_share_directory('tesikaga_lidar_detector'),
        'config',
        'detector_params.yaml'
    )
    
    # Launch引数として池の半径を受け取れるようにする
    pond_radius_arg = DeclareLaunchArgument(
        'pond_radius',
        default_value='3.0',
        description='Radius of the pond in meters for calibration.'
    )

    return LaunchDescription([
        pond_radius_arg,
        
        # 1. 検出ノード
        Node(
            package='tesikaga_lidar_detector',
            executable='detector_node',
            name='tesikaga_detector',
            parameters=[detector_config, {'calibration_mode': True}],
            output='screen'
        ),
        
        # 2. キャリブレーションノード
        Node(
            package='tesikaga_lidar_detector',
            executable='calibration_node',
            name='tesikaga_calibrator',
            parameters=[{'pond_radius': LaunchConfiguration('pond_radius')}],
            output='screen'
        ),
    ])