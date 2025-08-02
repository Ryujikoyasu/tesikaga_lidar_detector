import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
import numpy as np
import open3d as o3d
import time
import yaml
import os
import cv2
from ament_index_python.packages import get_package_share_directory

# プロジェクト内のモジュールをインポート
from .point_cloud_processor import PointCloudProcessor
from .ros_utils import pointcloud2_to_open3d, open3d_to_pointcloud2
from .udp_sender import UdpSender


class ObjectClusterDetectorNode(Node):
    """
    ROSの責務を管理するノード。
    点群処理のロジックはPointCloudProcessorに委譲する。
    """
    def __init__(self):
        super().__init__('tesikaga_detector')

        # パラメータを宣言し、YAMLから読み込む
        self._declare_parameters()
        params = self._get_parameters_as_dict()

        # 専門家クラスをインスタンス化
        self.processor = PointCloudProcessor(params, self.get_logger())
        self.udp_sender = UdpSender(params['target_ip'], params['target_port'], self.get_logger())
        
        self.transform_matrix = self._load_transform_matrix(params.get('transform_matrix_file'))

        # 背景キャプチャ用の状態変数
        self.is_capturing = False
        self.capture_start_time = 0
        self.captured_clouds = []
        
        # ROSインターフェース (Subscription, Publication, Service)
        self.sub = self.create_subscription(
            PointCloud2, '~/input/point_cloud', self.lidar_callback, 10)
        
        self.pub_centroids = self.create_publisher(PointCloud2, '~/output/centroids', 10)
        self.pub_debug_filtered = self.create_publisher(PointCloud2, '~/debug/filtered', 10)
        self.pub_debug_foreground = self.create_publisher(PointCloud2, '~/debug/foreground', 10)
        self.pub_debug_clustered = self.create_publisher(PointCloud2, '~/debug/clustered', 10)

        self.srv = self.create_service(
            Empty, '~/capture_background', self.capture_background_callback)
            
        self.get_logger().info("Object Cluster Detector Node has been started.")
        self.get_logger().info("Call '/tesikaga_detector/capture_background' service to begin detection.")

    def _declare_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_distance', 1.0), ('max_distance', 15.0),
                ('ground_removal_height', 0.15), ('voxel_leaf_size', 0.05),
                ('background_capture_duration', 3.0),
                ('background_subtraction_threshold', 0.1),
                ('cluster_tolerance', 0.5), ('min_cluster_size', 20), ('max_cluster_size', 800),
                ('target_ip', '127.0.0.1'), ('target_port', 9999),
                ('fixed_frame_id', 'world'),
                ('transform_matrix_file', 'transform_matrix.yaml')
            ])

    def _get_parameters_as_dict(self):
        param_keys = [name for name, _ in self._parameters.items()]
        return {key: self.get_parameter(key).get_parameter_value().double_value if self.get_parameter(key).type_ == rclpy.Parameter.Type.DOUBLE 
                     else self.get_parameter(key).get_parameter_value().integer_value if self.get_parameter(key).type_ == rclpy.Parameter.Type.INTEGER
                     else self.get_parameter(key).get_parameter_value().string_value
                     for key in param_keys}
                     
    def _load_transform_matrix(self, filename: str) -> Optional[np.ndarray]:
        """設定ファイルからアフィン変換行列を読み込む。"""
        if not filename:
            self.get_logger().warn("Transform matrix file not specified. No transformation will be applied.")
            return None
        
        package_share_directory = get_package_share_directory('tesikaga_lidar_detector')
        filepath = os.path.join(package_share_directory, 'config', filename)
        
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            matrix = np.array(data['transformation_matrix'], dtype=np.float32)
            self.get_logger().info(f"Successfully loaded transformation matrix from '{filepath}'")
            return matrix
        except Exception as e:
            self.get_logger().error(f"Failed to load transformation matrix from '{filepath}': {e}. No transformation will be applied.")
            return None
        
    def capture_background_callback(self, request, response):
        """背景キャプチャを開始するサービスコールバック。"""
        self.get_logger().info('Starting background capture...')
        self.is_capturing = True
        self.captured_clouds = []
        self.capture_start_time = time.time()
        return response

    def lidar_callback(self, msg: PointCloud2):
        """LiDARデータを受信するたびに呼び出されるメインの処理関数。"""
        # --- 背景キャプチャ中の処理 ---
        if self.is_capturing:
            duration = self.get_parameter('background_capture_duration').get_parameter_value().double_value
            if time.time() - self.capture_start_time < duration:
                pcd = pointcloud2_to_open3d(msg)
                pcd_downsampled = pcd.voxel_down_sample(self.get_parameter('voxel_leaf_size').get_parameter_value().double_value)
                self.captured_clouds.append(pcd_downsampled)
            else:
                self.is_capturing = False
                if self.captured_clouds:
                    # 蓄積した点群を結合して一つの背景モデルを作成
                    combined_pcd = o3d.geometry.PointCloud()
                    for pcd in self.captured_clouds:
                        combined_pcd += pcd
                    self.processor.set_background(combined_pcd)
                    self.get_logger().info("Background model captured successfully.")
                else:
                    self.get_logger().warn("Captured no points for background model.")
            return

        # --- 通常の検出処理 ---
        if self.processor.background_model is None:
            return

        pcd_raw = pointcloud2_to_open3d(msg)
        
        # コアロジックに処理を委譲
        centroids, debug_clouds = self.processor.process_frame(pcd_raw)

        # 結果の送信と可視化
        if centroids.size > 0:
            if self.transform_matrix is not None:
                # cv2.transformは(1, N, 2)の形状を要求する
                centroids_lidar_2d = centroids[:, :2].reshape(1, -1, 2)
                # アフィン変換を適用
                centroids_world_2d = cv2.transform(centroids_lidar_2d, self.transform_matrix)
                # (N, 2)の形状に戻す
                transformed_centroids = centroids_world_2d.reshape(-1, 2)
            else:
                # 変換行列がなければ、XY座標をそのまま使う
                transformed_centroids = centroids[:, :2]
            
            # 2. 変換後の座標をUDPで送信
            self.udp_sender.send_centroids(transformed_centroids)

            
            # 重心座標もPointCloud2としてPublishする
            pcd_centroids = o3d.geometry.PointCloud()
            pcd_centroids.points = o3d.utility.Vector3dVector(centroids)
            self.pub_centroids.publish(open3d_to_pointcloud2(pcd_centroids, msg.header.frame_id, msg.header.stamp))

        # デバッグ用トピックをPublish
        if 'filtered' in debug_clouds:
            self.pub_debug_filtered.publish(open3d_to_pointcloud2(debug_clouds['filtered'], msg.header.frame_id, msg.header.stamp))
        if 'foreground' in debug_clouds:
            self.pub_debug_foreground.publish(open3d_to_pointcloud2(debug_clouds['foreground'], msg.header.frame_id, msg.header.stamp))
        if 'clustered' in debug_clouds:
            self.pub_debug_clustered.publish(open3d_to_pointcloud2(debug_clouds['clustered'], msg.header.frame_id, msg.header.stamp))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectClusterDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.udp_sender.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()