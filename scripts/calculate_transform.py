#!/usr/bin/env python3
import numpy as np
import json
import yaml
import os
import sys
from ament_index_python.packages import get_package_share_directory

from tesikaga_lidar_detector.math_utils import solve_lidar_pose_from_vectors, get_affine_transform_matrix, order_points_by_geometry

def main():
    calib_file = '/home/ryuddi/ros2_ws/src/tesikaga_lidar_detector/config/calibration_points.json'
    
    if not os.path.exists(calib_file):
        print(f"Error: {calib_file} not found. Run capture_point command first.")
        sys.exit(1)
        
    try:
        with open(calib_file, 'r') as f:
            data = json.load(f)
    except (json.JSONDecodeError, FileNotFoundError):
        print(f"Error: {calib_file} is corrupted or empty. Run capture_point command first.")
        sys.exit(1)
        
    points = data.get('points', [])
    if len(points) != 3:
        print(f"Error: Expected 3 points, but found {len(points)}.")
        print("Run capture_point command to add more points.")
        sys.exit(1)
        
    print(f"Captured points: {points}")
    
    # 理想的なワールド座標（config/detector_params.yamlから）
    pts_world_ideal = np.array([[3.15, 0.0], [-3.15, 0.0], [0.0, 2.65]])
    pts_lidar = np.array(points)
    
    try:
        # 点を幾何学的に順序付け
        pts_lidar_ordered = order_points_by_geometry(pts_lidar)
        print(f"Ordered points: {pts_lidar_ordered}")
        
        # 変換を計算
        theta_rad, t_vec = solve_lidar_pose_from_vectors(pts_world_ideal, pts_lidar_ordered)
        transform_matrix = get_affine_transform_matrix(theta_rad, t_vec)
        
        # 結果を保存
        output_data = {
            'lidar_pose': {
                'position_xy': t_vec.tolist(),
                'rotation_z_deg': float(np.rad2deg(theta_rad))
            },
            'transformation_matrix': transform_matrix.tolist()
        }
        
        output_path = '/home/ryuddi/ros2_ws/src/tesikaga_lidar_detector/config/transform_matrix.yaml'
        with open(output_path, 'w') as f:
            yaml.dump(output_data, f, indent=2, sort_keys=False)
            
        print(f"✅ Transform matrix saved to {output_path}")
        print(f"LiDAR position: ({t_vec[0]:.3f}, {t_vec[1]:.3f})")
        print(f"LiDAR rotation: {np.rad2deg(theta_rad):.1f}°")
        
        # キャリブレーションポイントファイルを削除（次回用）
        # try:
        #     os.remove(calib_file)
        #     print("Calibration points cleared for next calibration.")
        # except FileNotFoundError:
        #     pass
        print("Calibration points preserved in:", calib_file)
        
    except Exception as e:
        print(f"Error during calculation: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()