# tesikaga_lidar_detector/math_utils.py

import numpy as np

def solve_lidar_pose_from_vectors(pts_world, pts_lidar):
    """3点の対応関係から、LiDARの姿勢（回転角と並進ベクトル）を計算する。"""
    # ワールド座標とLiDAR座標で、点0から点1へのベクトルを計算
    v_world = pts_world[1] - pts_world[0]
    v_lidar = pts_lidar[1] - pts_lidar[0]

    # 各ベクトルの角度をatan2で計算
    theta_world = np.arctan2(v_world[1], v_world[0])
    theta_lidar = np.arctan2(v_lidar[1], v_lidar[0])
    
    # 回転角は角度の差
    theta_rad = theta_world - theta_lidar
    
    # 計算した回転角から回転行列Rを作成
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    rotation_matrix = np.array([[c, -s], [s, c]])
    
    # 3点すべてで並進ベクトルを計算し、平均を取って堅牢性を高める
    T1 = pts_world[0] - rotation_matrix @ pts_lidar[0]
    T2 = pts_world[1] - rotation_matrix @ pts_lidar[1]
    T3 = pts_world[2] - rotation_matrix @ pts_lidar[2]
    translation_vector = (T1 + T2 + T3) / 3.0

    return theta_rad, translation_vector

def get_affine_transform_matrix(theta_rad, translation_vector):
    """姿勢からOpenCV用の2x3アフィン変換行列を生成する。"""
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    rotation_matrix = np.array([[c, -s], [s, c]])
    return np.hstack([rotation_matrix, translation_vector.reshape(2, 1)])

def order_points_by_geometry(pts_unordered: np.ndarray) -> np.ndarray:
    """
    3つの2D点を幾何学的性質に基づき、(R,0), (-R,0), (0,R) に対応する順序へ並べ替える。
    最も遠い2点を直径の両端とし、残りの点を頂点として扱う。
    """
    if not isinstance(pts_unordered, np.ndarray) or pts_unordered.shape != (3, 2):
        raise ValueError("Input must be a 3x2 NumPy array.")

    # 3点間の距離の二乗をすべて計算
    d_01_sq = np.sum((pts_unordered[0] - pts_unordered[1])**2)
    d_02_sq = np.sum((pts_unordered[0] - pts_unordered[2])**2)
    d_12_sq = np.sum((pts_unordered[1] - pts_unordered[2])**2)

    distances = {(0, 1): d_01_sq, (0, 2): d_02_sq, (1, 2): d_12_sq}
    
    # 最も遠い2点のインデックスを見つける (これが直径の両端)
    (idx_a, idx_b) = max(distances, key=distances.get)
    # 残りの1点が「てっぺん」の点
    idx_top = 3 - idx_a - idx_b

    # (-R,0)と(R,0)を区別するために外積のZ成分を利用する
    # 「てっぺん」から直径の両端へのベクトルを計算
    vec_top_to_a = pts_unordered[idx_a] - pts_unordered[idx_top]
    vec_top_to_b = pts_unordered[idx_b] - pts_unordered[idx_top]
    
    # 外積のZ成分を計算 (2Dベクトルの場合は vz = x1*y2 - y1*x2)
    cross_product_z = vec_top_to_a[0] * vec_top_to_b[1] - vec_top_to_a[1] * vec_top_to_b[0]
    
    # ワールド座標の (0,R)->(-R,0) と (0,R)->(R,0) のベクトルで考えると、
    # 外積は正になる。この幾何学的関係とLiDAR座標系を合わせる。
    if cross_product_z > 0:
        # Aが(-R,0)に対応し、Bが(R,0)に対応する
        ordered_indices = [idx_b, idx_a, idx_top] # (R,0), (-R,0), (0,R) の順
    else:
        # Bが(-R,0)に対応し、Aが(R,0)に対応する
        ordered_indices = [idx_a, idx_b, idx_top] # (R,0), (-R,0), (0,R) の順

    return pts_unordered[ordered_indices]
