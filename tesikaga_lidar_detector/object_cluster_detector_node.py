# object_cluster_detector_node.py (Mission 3 Complete)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
import open3d as o3d
import numpy as np

# ★タスク3：重心をPointCloud2に変換するために追加
from .ros_utils import pointcloud2_to_open3d, open3d_to_pointcloud2
from .point_cloud_processor import PointCloudProcessor
from .udp_sender import UdpSender

class ObjectClusterDetectorNode(Node):
    """
    ミッション3：背景をキャプチャし、動的な物体のみを検出する
    """
    def __init__(self):
        super().__init__('tesikaga_detector')

        self._declare_parameters()
        params = self._get_parameters_as_dict()

        self.processor = PointCloudProcessor(params, self.get_logger())
        self.udp_sender = UdpSender(params['target_ip'], params['target_port'], self.get_logger())
        
        self.is_capturing = False
        self.captured_point_clouds = []
        self.background_capture_timer = None
        
        self.sub = self.create_subscription(
            PointCloud2, 
            '~/input/point_cloud',
            self.lidar_callback, 
            10
        )
        
        # --- デバッグ用Publisher群 ---
        self.pub_debug_filtered = self.create_publisher(PointCloud2, '~/debug/filtered', 10)
        self.pub_debug_foreground = self.create_publisher(PointCloud2, '~/debug/foreground', 10)
        # ★タスク3：クラスタリング結果のPublisherを追加
        self.pub_debug_clustered = self.create_publisher(PointCloud2, '~/debug/clustered', 10)

        # --- ★タスク3：最終的な重心座標のPublisher ---
        self.pub_centroids = self.create_publisher(PointCloud2, '~/output/centroids', 10)

        self.capture_background_service = self.create_service(
            Empty,
            '~/capture_background',
            self.capture_background_callback
        )

        self.get_logger().info("Object Cluster Detector Node has been started.")
        self.get_logger().info("Publishing clustered cloud to '/tesikaga_detector/debug/clustered'.")
        self.get_logger().info("Publishing centroids to '/tesikaga_detector/output/centroids'.")

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
                ('transform_matrix_file', 'transform_matrix.yaml'),
                ('fixed_frame_id', 'world')
            ])
        
    def _get_parameters_as_dict(self):
        params_list = self.get_parameters_by_prefix('').values()
        return {p.name: p.value for p in params_list}

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
        voxel_size = self.get_parameter('voxel_leaf_size').get_parameter_value().double_value
        background_model = combined_pcd.voxel_down_sample(voxel_size) if voxel_size > 0 else combined_pcd
        self.processor.set_background(background_model)
        self.get_logger().info("Background model has been set.")
        self.captured_point_clouds = []

    def lidar_callback(self, msg: PointCloud2):
        try:
            pcd_raw = pointcloud2_to_open3d(msg)

            if self.is_capturing:
                voxel_size = self.get_parameter('voxel_leaf_size').get_parameter_value().double_value
                pcd_to_capture = pcd_raw.voxel_down_sample(voxel_size) if voxel_size > 0 else pcd_raw
                self.captured_point_clouds.append(pcd_to_capture)
                self.get_logger().info(f"Capturing... ({len(self.captured_point_clouds)} frames)", throttle_duration_sec=1.0)
                return

            # ★タスク3： process_frameの返り値をすべて受け取る
            centroids, sizes, debug_clouds = self.processor.process_frame(pcd_raw)

            # --- UDP送信処理 ---
            if centroids.size > 0:
                # [centroid_x, centroid_y, size] の形式にデータを整形
                data_to_send = np.hstack([
                    centroids[:, :2], # centroid_x, centroid_y
                    sizes.reshape(-1, 1) # size
                ])
                self.udp_sender.send_data(data_to_send.flatten())

            # --- デバッグ用Publish処理 ---
            if 'filtered' in debug_clouds and debug_clouds['filtered'].has_points():
                self.pub_debug_filtered.publish(open3d_to_pointcloud2(debug_clouds['filtered'], msg.header.frame_id, msg.header.stamp))
            
            if 'foreground' in debug_clouds and debug_clouds['foreground'].has_points():
                self.pub_debug_foreground.publish(open3d_to_pointcloud2(debug_clouds['foreground'], msg.header.frame_id, msg.header.stamp))

            # ★タスク3：クラスタリング結果をPublish
            if 'clustered' in debug_clouds and debug_clouds['clustered'].has_points():
                self.pub_debug_clustered.publish(open3d_to_pointcloud2(debug_clouds['clustered'], msg.header.frame_id, msg.header.stamp))

            # ★タスク3：重心をPointCloud2としてPublish
            if centroids.size > 0:
                # NumPy配列をOpen3D PointCloudに変換
                centroid_pcd = o3d.geometry.PointCloud()
                centroid_pcd.points = o3d.utility.Vector3dVector(centroids)
                # PointCloud2メッセージに変換してPublish
                centroid_msg = open3d_to_pointcloud2(centroid_pcd, msg.header.frame_id, msg.header.stamp)
                self.pub_centroids.publish(centroid_msg)

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