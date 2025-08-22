import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tesikaga_lidar_detector'),
        'config',
        'detector_params.yaml'
    )

    # Launch引数のデフォルト値を '/unilidar/cloud' に変更
    point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/unilidar/cloud', # <--- ここを変更！
        description='Topic name for the input PointCloud2'
    )

    return LaunchDescription([
        point_cloud_topic_arg,
        Node(
            package='tesikaga_lidar_detector',
            executable='detector_node',
            name='tesikaga_detector',
            parameters=[config, {'calibration_mode': True}], # <-- calibration_modeをTrueに
            remappings=[
                ('~/input/point_cloud', LaunchConfiguration('point_cloud_topic'))
            ],
            output='screen'
        ),
        Node(
            package='tesikaga_lidar_detector',
            executable='calibration_node',
            name='tesikaga_calibrator',
            parameters=[config],
            output='screen'
        )
    ])