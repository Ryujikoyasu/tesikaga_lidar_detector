# object_cluster_detector_node.py (Mission 3 Complete)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
import open3d as o3d
import numpy as np

# object_cluster_detector_node.py (Final Refactoring)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
import open3d as o3d
import numpy as np

from .ros_utils import pointcloud2_to_open3d, open3d_to_pointcloud2
from .point_cloud_processor import PointCloudProcessor
from .udp_sender import UdpSender

class ObjectClusterDetectorNode(Node):
    """
    モードに応じてパラメータを動的に切り替え、高精度と高パフォーマンスを両立する。
    """
    def __init__(self):
        super().__init__('tesikaga_detector')

        self._declare_parameters()

        # --- パラメータの読み込み ---
        self.is_calibration_mode = self.get_parameter('calibration_mode').get_parameter_value().bool_value
        
        # --- 本番用とキャリブレーション用のパラメータを完全に分離して保持 ---
        self.voxel_size_production = self.get_parameter('voxel_leaf_size').get_parameter_value().double_value
        self.voxel_size_calibration = self.get_parameter('calibration_voxel_leaf_size').get_parameter_value().double_value
        
        self.cluster_params_production = {
            'tolerance': self.get_parameter('cluster_tolerance').get_parameter_value().double_value,
            'min_size': self.get_parameter('min_cluster_size').get_parameter_value().integer_value,
            'max_size': self.get_parameter('max_cluster_size').get_parameter_value().integer_value,
        }
        self.cluster_params_calibration = {
            'tolerance': self.get_parameter('calibration_cluster.tolerance').get_parameter_value().double_value,
            'min_size': self.get_parameter('calibration_cluster.min_size').get_parameter_value().integer_value,
            'max_size': self.get_parameter('calibration_cluster.max_size').get_parameter_value().integer_value,
        }
        
        # --- フィルタリング等、モードに依存しない共通パラメータ ---
        self.filter_params = {
            'min_distance': self.get_parameter('min_distance').get_parameter_value().double_value,
            'max_distance': self.get_parameter('max_distance').get_parameter_value().double_value,
            'ground_removal_height': self.get_parameter('ground_removal_height').get_parameter_value().double_value,
            'background_subtraction_threshold': self.get_parameter('background_subtraction_threshold').get_parameter_value().double_value,
        }
        self.get_logger().info(f"Loaded max_distance: {self.filter_params['max_distance']}")
        
        # --- ProcessorとSenderの初期化 ---
        self.processor = PointCloudProcessor(self.get_logger())
        self.udp_sender = UdpSender(
            self.get_parameter('target_ip').get_parameter_value().string_value,
            self.get_parameter('target_port').get_parameter_value().integer_value,
            self.get_logger()
        )
        
        # --- 前景点群の蓄積用設定 ---
        self.accumulation_duration = self.get_parameter('accumulation_duration').get_parameter_value().double_value
        self.foreground_buffer = []
        self.last_process_time = self.get_clock().now()
        
        self.is_capturing = False
        self.captured_point_clouds = []
        self.background_capture_timer = None
        
        self.sub = self.create_subscription(PointCloud2, '~/input/point_cloud', self.lidar_callback, 10)
        
        self.pub_debug_filtered = self.create_publisher(PointCloud2, '~/debug/filtered', 10)
        self.pub_debug_foreground = self.create_publisher(PointCloud2, '~/debug/foreground', 10)
        self.pub_debug_clustered = self.create_publisher(PointCloud2, '~/debug/clustered', 10)
        self.pub_centroids = self.create_publisher(PointCloud2, '~/output/centroids', 10)

        self.capture_background_service = self.create_service(Empty, '~/capture_background', self.capture_background_callback)

        self.get_logger().info("Object Cluster Detector Node has been started.")
        self.get_logger().info(f"Calibration mode: {self.is_calibration_mode}")
        self.get_logger().info(f"Production Voxel Size: {self.voxel_size_production}, Calibration Voxel Size: {self.voxel_size_calibration}")


    def _declare_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_distance', 0.3), ('max_distance', 10.0),
                ('ground_removal_height', 0.15),
                ('background_capture_duration', 3.0),
                ('background_subtraction_threshold', 0.1),
                ('target_ip', '127.0.0.1'), ('target_port', 9999),
                ('transform_matrix_file', 'transform_matrix.yaml'),
                ('fixed_frame_id', 'world'),
                ('calibration_mode', False),
                ('accumulation_duration', 1.0),
                # --- 本番用パラメータ ---
                ('voxel_leaf_size', 0.05),
                ('cluster_tolerance', 0.5),
                ('min_cluster_size', 20),
                ('max_cluster_size', 800),
                # --- キャリブレーション用パラメータ ---
                ('calibration_voxel_leaf_size', 0.03),
            ])
        # --- キャリブレーション用クラスタパラメータ (ネスト) ---
        self.declare_parameter('calibration_cluster.min_size', 5)
        self.declare_parameter('calibration_cluster.max_size', 25)
        self.declare_parameter('calibration_cluster.tolerance', 0.1)
        
    def capture_background_callback(self, request, response):
        self.get_logger().info("Starting background capture...")
        self.is_capturing = True
        self.captured_point_clouds = []
        duration = self.get_parameter('background_capture_duration').get_parameter_value().double_value
        if self.background_capture_timer is not None:
            self.background_capture_timer.cancel()
        self.background_capture_timer = self.create_timer(duration, self.process_captured_background_callback)
        self.get_logger().info(f"Will capture background for {duration} seconds.")
        return response

    def process_captured_background_callback(self):
        if self.background_capture_timer is not None:
            self.background_capture_timer.cancel()
            self.background_capture_timer = None
        self.is_capturing = False
        self.get_logger().info(f"Finished capturing. Processing {len(self.captured_point_clouds)} point clouds.")
        if not self.captured_point_clouds:
            self.get_logger().warn("No point clouds were captured.")
            return
        
        combined_pcd = o3d.geometry.PointCloud()
        for pcd in self.captured_point_clouds:
            combined_pcd += pcd
        
        # 背景モデルは常に本番用の解像度で作成する
        background_model = combined_pcd.voxel_down_sample(self.voxel_size_production)
        self.processor.set_background(background_model)
        self.get_logger().info("Background model has been set.")
        self.captured_point_clouds = []

    def lidar_callback(self, msg: PointCloud2):
        try:
            pcd_raw = pointcloud2_to_open3d(msg)

            # 背景キャプチャモードの場合は、以前のロジックを実行
            if self.is_capturing:
                pcd_to_capture = pcd_raw.voxel_down_sample(self.voxel_size_production)
                self.captured_point_clouds.append(pcd_to_capture)
                self.get_logger().info(f"Capturing... ({len(self.captured_point_clouds)} frames)", throttle_duration_sec=1.0)
                return

            # --- 1. 前景抽出 ---
            active_voxel_size = self.voxel_size_calibration if self.is_calibration_mode else self.voxel_size_production
            
            foreground_pcd, debug_pre_clouds = self.processor.preprocess_and_get_foreground(
                pcd_raw, active_voxel_size, self.filter_params
            )

            if foreground_pcd.has_points():
                self.foreground_buffer.append(foreground_pcd)

            # --- 2. 時間ベースでの蓄積と処理 ---
            now = self.get_clock().now()
            if (now - self.last_process_time).nanoseconds / 1e9 < self.accumulation_duration:
                return # まだ処理する時間ではない

            self.last_process_time = now # 処理時間を更新

            if not self.foreground_buffer:
                return

            # --- 3. 蓄積した点群を結合 ---
            combined_pcd = o3d.geometry.PointCloud()
            for pcd in self.foreground_buffer:
                combined_pcd += pcd
            self.foreground_buffer = [] # バッファをクリア

            self.get_logger().info(f"Processing {len(combined_pcd.points)} accumulated points.")

            # --- 4. クラスタリング ---
            active_cluster_params = self.cluster_params_calibration if self.is_calibration_mode else self.cluster_params_production
            centroids, sizes, debug_cluster_clouds = self.processor.find_clusters(combined_pcd, active_cluster_params)

            # --- 5. UDP送信とデバッグ情報のPublish ---
            if centroids.size > 0:
                data_to_send = np.hstack([centroids[:, :2], sizes.reshape(-1, 1)])
                self.udp_sender.send_data(data_to_send.flatten())

            header = msg.header
            if 'filtered' in debug_pre_clouds and debug_pre_clouds['filtered'].has_points():
                self.pub_debug_filtered.publish(open3d_to_pointcloud2(debug_pre_clouds['filtered'], header.frame_id, header.stamp))
            if 'foreground' in debug_pre_clouds and debug_pre_clouds['foreground'].has_points():
                self.pub_debug_foreground.publish(open3d_to_pointcloud2(debug_pre_clouds['foreground'], header.frame_id, header.stamp))
            
            if 'clustered' in debug_cluster_clouds and debug_cluster_clouds['clustered'].has_points():
                self.pub_debug_clustered.publish(open3d_to_pointcloud2(debug_cluster_clouds['clustered'], header.frame_id, header.stamp))
            if centroids.size > 0:
                centroid_pcd = o3d.geometry.PointCloud()
                centroid_pcd.points = o3d.utility.Vector3dVector(centroids)
                self.pub_centroids.publish(open3d_to_pointcloud2(centroid_pcd, header.frame_id, header.stamp))

        except Exception as e:
            self.get_logger().error(f"Error in lidar_callback: {e}", exc_info=True)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectClusterDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.udp_sender:
            node.udp_sender.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


