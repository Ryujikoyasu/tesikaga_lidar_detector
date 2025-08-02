import numpy as np
import cv2
import yaml
import os

def calculate_and_save_transform():
    """
    既知の3点の対応関係からアフィン変換行列を計算し、YAMLファイルに保存する。
    """
    # --- ステップ1: 測定した座標をここにコピペする ---

    # ワールド座標系でのマーカーの既知の座標 (物理的に測定した値)
    # 例: (2,0), (0,2), (-1,-1)
    pts_world = np.float32([
        [2.0, 0.0],
        [0.0, 2.0],
        [-1.0, -1.0]
    ])

    # LiDAR座標系で測定されたマーカーの座標 (ros2 topic echoでメモした値)
    pts_lidar = np.float32([
        [1.53, -2.18], # マーカー1のLiDAR座標
        [3.41, -0.25], # マーカー2のLiDAR座標
        [2.11, -3.89]  # マーカー3のLiDAR座標
    ])

    # --- ステップ2: 変換行列の計算 ---
    
    # LiDAR座標系 -> ワールド座標系へのアフィン変換行列を計算
    # (入力がpts_lidar, 出力がpts_world)
    transform_matrix = cv2.getAffineTransform(pts_lidar, pts_world)
    
    print("計算された変換行列:")
    print(transform_matrix)

    # --- ステップ3: 行列をファイルに保存 ---

    # 保存先のパスを決定 (configディレクトリ)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    output_path = os.path.join(package_dir, 'config', 'transform_matrix.yaml')

    matrix_list = transform_matrix.tolist()

    with open(output_path, 'w') as f:
        yaml.dump({'transformation_matrix': matrix_list}, f)

    print(f"\n変換行列が '{output_path}' に保存されました。")
    print("detector_params.yamlにこのファイル名を追加してください。")

if __name__ == '__main__':
    calculate_and_save_transform()