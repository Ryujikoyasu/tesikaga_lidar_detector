#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import json
import os
import sys
from ament_index_python.packages import get_package_share_directory

from tesikaga_lidar_detector.ros_utils import pointcloud2_to_open3d

class PointCapture(Node):
    def __init__(self):
        super().__init__('point_capture')
        self.captured = False
        self.centroid_sub = self.create_subscription(
            PointCloud2,
            '/tesikaga_detector/output/centroids',
            self.centroid_callback,
            10
        )
        self.get_logger().info("Waiting for centroids...")
        
    def centroid_callback(self, msg):
        if self.captured:
            return
            
        pcd = pointcloud2_to_open3d(msg)
        points = np.asarray(pcd.points)[:, :2]  # Z座標は無視
        
        if len(points) != 1:
            self.get_logger().warn(f"Expected 1 person, but found {len(points)}. Please stand alone at marker position.")
            return
            
        point = points[0]
        self.get_logger().info(f"Captured point: ({point[0]:.3f}, {point[1]:.3f})")
        
        # ファイルに保存（分かりやすい場所に）
        calib_file = '/home/ryuddi/ros2_ws/src/tesikaga_lidar_detector/config/calibration_points.json'
        
        # 既存の点があれば読み込み
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    data = json.load(f)
            except (json.JSONDecodeError, FileNotFoundError):
                self.get_logger().warn("Calibration file corrupted or empty. Starting fresh.")
                data = {'points': []}
        else:
            data = {'points': []}
            
        # 新しい点を追加
        data['points'].append([float(point[0]), float(point[1])])
        
        with open(calib_file, 'w') as f:
            json.dump(data, f, indent=2)
            
        num_points = len(data['points'])
        self.get_logger().info(f"Point {num_points}/3 saved to {calib_file}")
        
        if num_points >= 3:
            self.get_logger().info("All 3 points captured! Run calculate_transform command next.")
            
        self.captured = True

def main():
    rclpy.init()
    node = PointCapture()
    
    try:
        while not node.captured and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()