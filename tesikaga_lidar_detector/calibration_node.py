import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger as TriggerService # 名前が重複するので別名でインポート
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from .ros_utils import pointcloud2_to_open3d
from .math_utils import solve_lidar_pose_from_vectors, get_affine_transform_matrix, order_points_by_geometry
# from tesikaga_lidar_detector.srv import TriggerCalibration

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('tesikaga_calibrator')

        self.declare_parameter('calibration_markers', rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.latest_centroids = np.array([])
        self.calibration_points = []

        try:
            marker_values = self.get_parameter('calibration_markers').get_parameter_value().double_array_value
            self.pts_world_ideal = np.array(marker_values).reshape(-1, 2)
            self.get_logger().info(f"Ideal world marker points loaded:\n{self.pts_world_ideal}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to get 'calibration_markers': {e}. Shutting down.")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.centroid_sub = self.create_subscription(
            PointCloud2,
            '/tesikaga_detector/output/centroids',
            self.centroid_callback,
            10
        )

        # サービスの定義
        self.add_point_service = self.create_service(
            TriggerService,
            '~/add_calibration_point',
            self.add_point_callback
        )
        self.calculate_service = self.create_service(
            TriggerService,
            '~/calculate_transform',
            self.calculate_transform_callback
        )
        self.clear_service = self.create_service(
            TriggerService,
            '~/clear_calibration',
            self.clear_callback
        )

        self.get_logger().info("Calibration Node has been started.")
        self.get_logger().info("Ready to add calibration points.")

    def centroid_callback(self, msg):
        pcd = pointcloud2_to_open3d(msg)
        self.latest_centroids = np.asarray(pcd.points)[:, :2] # Z座標は無視

    def add_point_callback(self, request, response):
        if len(self.calibration_points) >= 3:
            response.success = False
            response.message = "Error: 3 points have already been registered. Please calculate or clear."
            self.get_logger().warn(response.message)
            return response

        if len(self.latest_centroids) != 1:
            response.success = False
            response.message = f"Error: Expected 1 person (marker), but found {len(self.latest_centroids)}. Please stand alone in the marker position."
            self.get_logger().warn(response.message)
            return response

        new_point = self.latest_centroids[0]
        self.calibration_points.append(new_point)
        num_points = len(self.calibration_points)
        response.success = True
        response.message = f"Successfully added point {num_points}/3. Position: ({new_point[0]:.2f}, {new_point[1]:.2f})"
        self.get_logger().info(response.message)
        return response

    def clear_callback(self, request, response):
        self.calibration_points = []
        response.success = True
        response.message = "Cleared all calibration points."
        self.get_logger().info(response.message)
        return response

    def calculate_transform_callback(self, request, response):
        if len(self.calibration_points) != 3:
            response.success = False
            response.message = f"Error: Expected 3 points, but {len(self.calibration_points)} are registered."
            self.get_logger().error(response.message)
            return response

        try:
            pts_lidar_ordered = order_points_by_geometry(np.array(self.calibration_points))
            theta_rad, t_vec = solve_lidar_pose_from_vectors(self.pts_world_ideal, pts_lidar_ordered)
            transform_matrix = get_affine_transform_matrix(theta_rad, t_vec)

            output_data = {
                'lidar_pose': {
                    'position_xy': t_vec.tolist(),
                    'rotation_z_deg': np.rad2deg(theta_rad)
                },
                'transformation_matrix': transform_matrix.tolist()
            }

            package_path = get_package_share_directory('tesikaga_lidar_detector')
            output_path = os.path.join(package_path, 'config', 'transform_matrix.yaml')

            with open(output_path, 'w') as f:
                yaml.dump(output_data, f, indent=2, sort_keys=False)

            response.success = True
            response.message = (f"Calibration successful! Transform matrix saved to '{output_path}'")
            self.get_logger().info(response.message)
            
            # 計算後はリセット
            self.calibration_points = []

        except Exception as e:
            response.success = False
            response.message = f"An unexpected error occurred during calculation: {e}"
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
