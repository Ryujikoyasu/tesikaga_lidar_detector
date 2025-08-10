# tesikaga_lidar_detector/calibration_node.py

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from .ros_utils import pointcloud2_to_open3d
from .math_utils import solve_lidar_pose_from_vectors, get_affine_transform_matrix, order_points_by_geometry
from tesikaga_lidar_detector.srv import TriggerCalibration

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('tesikaga_calibrator')
        
        # パラメータ宣言
        self.declare_parameter('pond_radius', 3.0)
        # ★ ROS2のYAMLパーサーの制約を回避するため、フラットなリストとして読み込む
        self.declare_parameter('calibration_markers', rclpy.Parameter.Type.DOUBLE_ARRAY)
        
        # 変数初期化
        self.latest_centroids = np.array([])
        self.pond_radius = self.get_parameter('pond_radius').get_parameter_value().double_value
        
        # パラメータから理想座標を読み込む
        try:
            # フラットなリスト [x1, y1, x2, y2, ...] を (N, 2) の形状に変換
            marker_values = self.get_parameter('calibration_markers').get_parameter_value().double_array_value
            self.pts_world_ideal = np.array(marker_values).reshape(-1, 2)
            self.get_logger().info(f"Ideal world marker points loaded from params:\n{self.pts_world_ideal}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to get or parse 'calibration_markers' parameter: {e}")
            self.get_logger().fatal("This is a critical error. Please check your detector_params.yaml. Shutting down.")
            # 致命的なエラーなのでノードを終了させる
            self.destroy_node()
            rclpy.shutdown()
            return

        # サブスクライバ
        self.centroid_sub = self.create_subscription(
            PointCloud2,
            '/tesikaga_detector/output/centroids',
            self.centroid_callback,
            10
        )
        
        # サービス
        self.calibration_service = self.create_service(
            TriggerCalibration,
            '~/trigger_calibration',
            self.trigger_calibration_callback
        )
        
        self.get_logger().info("Calibration Node has been started.")
        self.get_logger().info(f"Waiting for calibration trigger. Pond radius set to {self.pond_radius}m.")

    def centroid_callback(self, msg):
        pcd = pointcloud2_to_open3d(msg)
        self.latest_centroids = np.asarray(pcd.points)[:, :2] # Z座標は無視

    def trigger_calibration_callback(self, request, response):
        # 競合状態を避けるため、計算開始時のデータをコピーして使用
        points_to_process = self.latest_centroids.copy()

        if len(points_to_process) != 3:
            response.success = False
            response.message = f"Error: Expected 3 markers, but found {len(points_to_process)}. Please check LiDAR view."
            self.get_logger().error(response.message)
            return response

        try:
            # 数学ロジックを呼び出し、LiDAR座標点を並べ替える
            pts_lidar_ordered = order_points_by_geometry(points_to_process)
            
            # 姿勢を計算
            theta_rad, t_vec = solve_lidar_pose_from_vectors(self.pts_world_ideal, pts_lidar_ordered)
            
            # アフィン変換行列を生成
            transform_matrix = get_affine_transform_matrix(theta_rad, t_vec)
            
            # --- ★リファクタリングポイント ---
            # YAMLに保存するデータを構築
            output_data = {
                'lidar_pose': {
                    'position_xy': t_vec.tolist(),
                    'rotation_z_deg': np.rad2deg(theta_rad)
                },
                'transformation_matrix': transform_matrix.tolist()
            }
            
            # ファイルに保存
            package_path = get_package_share_directory('tesikaga_lidar_detector')
            output_path = os.path.join(package_path, 'config', 'transform_matrix.yaml')
            
            with open(output_path, 'w') as f:
                # yaml.dump に indent をつけると、より人間が読みやすい形式になる
                yaml.dump(output_data, f, indent=2, sort_keys=False)

            response.success = True
            response.message = (f"Calibration successful! LiDAR Pose calculated:\n"
                                f"  Position (x,y): ({t_vec[0]:.3f}, {t_vec[1]:.3f}) m\n"
                                f"  Rotation (Z): {np.rad2deg(theta_rad):.3f} degrees\n"
                                f"Transformation matrix saved to '{output_path}'")
            self.get_logger().info(response.message)

        except ValueError as ve:
            response.success = False
            response.message = f"Math error during calibration: {ve}"
            self.get_logger().error(response.message, exc_info=True)
        except Exception as e:
            response.success = False
            response.message = f"An unexpected error occurred during calibration: {e}"
            self.get_logger().error(response.message, exc_info=True)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()