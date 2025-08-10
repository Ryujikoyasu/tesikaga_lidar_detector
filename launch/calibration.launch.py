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
    
    return LaunchDescription([
        Node(
            package='tesikaga_lidar_detector',
            executable='detector_node',
            name='tesikaga_detector',
            parameters=[detector_config],
            output='screen'
        ),
        
        # 2. キャリブレーションノード (検出ノードと同じ設定ファイルを読み込む)
        Node(
            package='tesikaga_lidar_detector',
            executable='calibration_node',
            name='tesikaga_calibrator',
            parameters=[detector_config],
            output='screen'
        ),
    ])