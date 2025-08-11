# tesikaga-art/calibrate.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
import yaml
import threading
import time
import os
import struct # PointCloud2のデコードに必要

# -------------------------------------------------------------------
# 0. ユーティリティ関数 (ros_utils.pyのロジックを参考)
# -------------------------------------------------------------------
def pointcloud2_to_numpy(msg: PointCloud2) -> np.ndarray:
    """
    sensor_msgs/PointCloud2をNumpy配列(Nx3)に変換する。
    x, y, zフィールドのオフセットを動的に探し、他のフィールドは無視する。
    """
    offsets = {f.name: f.offset for f in msg.fields}
    if 'x' not in offsets or 'y' not in offsets or 'z' not in offsets:
        # 必要なフィールドが見つからない場合は空の配列を返す
        return np.empty((0, 3), dtype=np.float32)

    point_step = msg.point_step
    num_points = msg.width * msg.height
    
    xyz = np.zeros((num_points, 3), dtype=np.float32)
    
    data = msg.data
    for i in range(num_points):
        base_idx = i * point_step
        # struct.unpack_fromを使用して、バッファの特定オフセットからfloatを読み取る
        xyz[i, 0] = struct.unpack_from('<f', data, base_idx + offsets['x'])[0]
        xyz[i, 1] = struct.unpack_from('<f', data, base_idx + offsets['y'])[0]
        xyz[i, 2] = struct.unpack_from('<f', data, base_idx + offsets['z'])[0]
        
    return xyz

# -------------------------------------------------------------------
# 1. 状態管理クラス (State) - SRP: 状態を保持するだけの存在
# -------------------------------------------------------------------
class CalibrationState:
    """キャリブレーションの進行状況とデータを一元管理する"""
    def __init__(self, target_positions_world):
        self.lock = threading.Lock()
        self.target_positions_world = target_positions_world
        self.measured_positions_lidar = {} # {marker_id: np.array([x, y])}
        self.latest_centroids_lidar = np.empty((0, 2)) # LiDAR座標系の重心

    def set_latest_centroids(self, centroids: np.ndarray):
        with self.lock:
            self.latest_centroids_lidar = centroids[:, :2] # Z軸は無視

    def record_marker_position(self, marker_id: int):
        with self.lock:
            # 現在検出されている重心が1つだけの場合、それをマーカーの座標として記録
            if self.latest_centroids_lidar.shape[0] == 1:
                self.measured_positions_lidar[marker_id] = self.latest_centroids_lidar[0]
                return True, self.latest_centroids_lidar[0]
            else:
                return False, None

    def is_ready_for_calculation(self) -> bool:
        with self.lock:
            return len(self.measured_positions_lidar) == len(self.target_positions_world)

# -------------------------------------------------------------------
# 2. ROS2通信クラス (Listener) - SRP: ROS2からの受信に専念する
# -------------------------------------------------------------------
class CentroidListener(threading.Thread):
    """別スレッドでROS2トピックを購読し、最新の重心情報をStateに渡す"""
    def __init__(self, node: Node, state: CalibrationState):
        super().__init__(daemon=True)
        self.node = node
        self.state = state
        self.subscriber = self.node.create_subscription(
            PointCloud2,
            '/tesikaga_detector/output/centroids', # detector_nodeが出力する重心トピック
            self._centroid_callback,
            10
        )

    def _centroid_callback(self, msg: PointCloud2):
        # PointCloud2からNumpy配列への変換処理
        centroids_xyz = pointcloud2_to_numpy(msg)
        if centroids_xyz.size > 0:
            self.state.set_latest_centroids(centroids_xyz)

    def run(self):
        rclpy.spin(self.node)

# -------------------------------------------------------------------
# 3. 制御クラス (Controller) - SRP: ユーザーとの対話と全体の流れを制御
# -------------------------------------------------------------------
class CalibrationController:
    """ユーザーとの対話を通じてキャリブレーションプロセス全体を管理する"""
    def __init__(self):
        # --- 設定値 ---
        POND_RADIUS = 2.5 # 例: 池の半径が2.5mの場合
        self.MARKER_POSITIONS_WORLD = {
            1: np.float32([POND_RADIUS, 0.0]),
            2: np.float32([-POND_RADIUS, 0.0]),
            3: np.float32([0.0, POND_RADIUS])
        }
        self.state = CalibrationState(self.MARKER_POSITIONS_WORLD)
        self.node = Node('calibration_controller')
        self.listener_thread = CentroidListener(self.node, self.state)

    def run_calibration_flow(self):
        """キャリブレーションの対話フローを実行する"""
        print("--- Tesikaga Art: Daily Calibration System v2.0 ---")
        print("INFO: Background subtractionサービスを呼び出し、背景をキャプチャしてください。")
        print("INFO: (別ターミナルで `ros2 service call /tesikaga_detector/capture_background std_srvs/srv/Empty` を実行)")
        input("背景キャプチャが完了したら、Enterキーを押してください...")

        self.listener_thread.start()
        
        for marker_id, world_pos in self.MARKER_POSITIONS_WORLD.items():
            print("\n----------------------------------------------------")
            print(f"STEP {marker_id}: マーカー {marker_id} を配置してください。")
            print(f"目標ワールド座標: ({world_pos[0]:.2f}, {world_pos[1]:.2f})")
            
            # ボトルを1本だけ置いてもらう
            input("マーカー以外の動く物体がないことを確認し、Enterを押して座標を記録します...")
            
            time.sleep(1.0) # 最新の座標を受信するための短い待機
            
            success, measured_pos = self.state.record_marker_position(marker_id)
            if success:
                print(f"OK: マーカー {marker_id} のLiDAR座標を記録しました: ({measured_pos[0]:.2f}, {measured_pos[1]:.2f})")
            else:
                print(f"ERROR: マーカーを1つだけ検出できませんでした。検出数: {self.state.latest_centroids_lidar.shape[0]}")
                print("最初からやり直してください。")
                self.node.destroy_node()
                return

        if self.state.is_ready_for_calculation():
            self.calculate_and_save_transform()
        else:
            print("ERROR: 全てのマーカー座標を記録できませんでした。")
        
        self.node.destroy_node()

    def calculate_and_save_transform(self):
        """3点の対応関係からアフィン変換行列を計算し、ファイルに保存する"""
        print("\n----------------------------------------------------")
        print("SUCCESS: 3点全ての座標を記録しました。変換行列を計算します...")

        pts_world = np.array(list(self.state.target_positions_world.values()), dtype=np.float32)
        pts_lidar = np.array(list(self.state.measured_positions_lidar.values()), dtype=np.float32)

        if pts_lidar.shape != (3, 2) or pts_world.shape != (3, 2):
            print(f"ERROR: 座標の形式が正しくありません。LiDAR={pts_lidar.shape}, World={pts_world.shape}")
            return

        transform_matrix = cv2.getAffineTransform(pts_lidar, pts_world)

        print("計算された変換行列 (LiDAR -> World):")
        print(transform_matrix)
        
        project_root = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(project_root, "config")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, 'transform_matrix.yaml')

        data_to_save = {'transformation_matrix': transform_matrix.tolist()}

        with open(output_path, 'w') as f:
            yaml.dump(data_to_save, f, default_flow_style=False)

        print(f"\n変換行列が '{output_path}' に保存されました。")
        print("キャリブレーションが完了しました！")

def main():
    rclpy.init()
    controller = CalibrationController()
    try:
        controller.run_calibration_flow()
    except KeyboardInterrupt:
        print("\nキャリブレーションが中断されました。")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("システムをシャットダウンしました。")

if __name__ == '__main__':
    main()
