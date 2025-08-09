import open3d as o3d
import numpy as np
from typing import Optional, List, Dict

class PointCloudProcessor:
    """
    点群処理のコアロジックを責務として持つ。
    ROSに依存せず、Open3DとNumPyのみで完結する。
    """
    def __init__(self, params: dict, logger):
        self._params = params
        self._logger = logger
        self.background_model: Optional[o3d.geometry.PointCloud] = None
        self._logger.info("Point cloud processor initialized.")
        self._logger.info(f"Parameters: {self._params}")

    def set_background(self, pcd: o3d.geometry.PointCloud):
        """背景モデルを設定する。"""
        self.background_model = pcd
        self._logger.info(f"Background model set with {len(pcd.points)} points.")

    def process_frame(self, pcd: o3d.geometry.PointCloud, cluster_params: dict) -> (np.ndarray, np.ndarray, Dict[str, o3d.geometry.PointCloud]):
        """
        単一フレームの点群を処理し、重心座標とデバッグ用点群を返す。
        """
        debug_clouds = {}

        # 1. ダウンサンプリング
        pcd_downsampled = pcd.voxel_down_sample(self._params['voxel_leaf_size'])
        debug_clouds['downsampled'] = pcd_downsampled

        # 2. 領域フィルタリング
        pcd_filtered = self._filter_by_distance_and_height(pcd_downsampled)
        debug_clouds['filtered'] = pcd_filtered

        if self.background_model is None:
            return np.array([]), np.array([]), debug_clouds

        # 3. 背景差分
        pcd_foreground = self._subtract_background(pcd_filtered)
        debug_clouds['foreground'] = pcd_foreground

        if not pcd_foreground.has_points():
            return np.array([]), np.array([]), debug_clouds

        # 4. クラスタリング
        centroids, sizes, clustered_cloud = self._get_clusters(pcd_foreground)
        debug_clouds['clustered'] = clustered_cloud

        return centroids, sizes, debug_clouds

    def _filter_by_distance_and_height(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """距離と高さで点群をフィルタリングする。"""
        points = np.asarray(pcd.points)
        
        # 中心からの距離を計算
        distances = np.linalg.norm(points[:, :2], axis=1) # XY平面での距離
        
        # フィルタリング条件
        dist_mask = (distances >= self._params['min_distance']) & (distances <= self._params['max_distance'])
        height_mask = points[:, 2] > self._params['ground_removal_height']
        
        combined_mask = dist_mask & height_mask
        
        return pcd.select_by_index(np.where(combined_mask)[0])

    def _subtract_background(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """背景モデルとの距離を計算し、前景を抽出する。"""
        if self.background_model is None or not pcd.has_points():
            return o3d.geometry.PointCloud()

        # 各前景点が背景モデルのどのくらい近くにあるかを計算
        dists = pcd.compute_point_cloud_distance(self.background_model)
        dists = np.asarray(dists)
        
        # 閾値より遠い点（=前景）のみを抽出
        foreground_indices = np.where(dists > self._params['background_subtraction_threshold'])[0]
        
        return pcd.select_by_index(foreground_indices)

    def _get_clusters(self, pcd: o3d.geometry.PointCloud, cluster_params: dict) -> (np.ndarray, np.ndarray, o3d.geometry.PointCloud):
        """DBSCANでクラスタリングし、有効なクラスタの重心とサイズを計算する。"""
        # DBSCANを実行
        labels = np.array(pcd.cluster_dbscan(
            eps=cluster_params['tolerance'],
            min_points=cluster_params['min_size'],
            print_progress=False
        ))

        max_label = labels.max()
        if max_label < 0:
            return np.array([]), np.array([]), o3d.geometry.PointCloud()

        # --- Matplotlib依存の排除 ---
        # 固定カラーマップを定義
        DEBUG_COLORS = np.array([
            [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 0.0],
            [0.0, 1.0, 1.0], [1.0, 0.0, 1.0], [0.8, 0.6, 0.4], [0.6, 0.8, 0.4],
            [0.4, 0.6, 0.8], [0.8, 0.4, 0.6], [1.0, 0.5, 0.0], [0.0, 0.5, 1.0],
            [0.5, 1.0, 0.0], [0.5, 0.0, 1.0], [1.0, 0.0, 0.5], [0.0, 1.0, 0.5],
            [0.7, 0.7, 0.7], [0.5, 0.2, 0.2], [0.2, 0.5, 0.2], [0.2, 0.2, 0.5]
        ])
        
        # 各点に色を割り当て (ノイズは黒)
        colors = np.zeros((len(labels), 3))
        non_noise_indices = labels >= 0
        if np.any(non_noise_indices):
            color_indices = labels[non_noise_indices] % len(DEBUG_COLORS)
            colors[non_noise_indices] = DEBUG_COLORS[color_indices]
        # --------------------------

        clustered_cloud = o3d.geometry.PointCloud()
        clustered_cloud.points = pcd.points
        clustered_cloud.colors = o3d.utility.Vector3dVector(colors)

        # 有効なクラスタの重心とサイズを計算
        valid_centroids = []
        valid_sizes = []
        unique_labels = np.unique(labels)
        for label in unique_labels:
            if label == -1: continue # -1はノイズ

            cluster_indices = np.where(labels == label)[0]
            
            # クラスタサイズのフィルタリング
            if cluster_params['min_size'] <= len(cluster_indices) <= cluster_params['max_size']:
                cluster_points = np.asarray(pcd.points)[cluster_indices]
                centroid = np.mean(cluster_points, axis=0)
                
                # AABB (Axis-Aligned Bounding Box) からサイズを計算
                aabb = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(cluster_points))
                extent = aabb.get_extent()
                size = np.linalg.norm(extent[:2]) # XY平面の対角線の長さをサイズとする

                valid_centroids.append(centroid)
                valid_sizes.append(size)
        
        return np.array(valid_centroids), np.array(valid_sizes), clustered_cloud