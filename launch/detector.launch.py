import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 設定ファイルのパスを取得
    config = os.path.join(
        get_package_share_directory('tesikaga_lidar_detector'),
        'config',
        'detector_params.yaml'
    )

    # Launch引数を宣言
    point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='point_cloud',
        description='Topic name for the input PointCloud2'
    )

    return LaunchDescription([
        point_cloud_topic_arg,
        Node(
            package='tesikaga_lidar_detector',
            executable='detector_node',
            name='tesikaga_detector',
            parameters=[config],
            remappings=[
                ('~/input/point_cloud', LaunchConfiguration('point_cloud_topic'))
            ],
            output='screen'
        )
    ])