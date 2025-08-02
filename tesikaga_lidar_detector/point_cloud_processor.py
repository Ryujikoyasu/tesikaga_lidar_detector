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

    def process_frame(self, pcd: o3d.geometry.PointCloud) -> (np.ndarray, Dict[str, o3d.geometry.PointCloud]):
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
            return np.array([]), debug_clouds

        # 3. 背景差分
        pcd_foreground = self._subtract_background(pcd_filtered)
        debug_clouds['foreground'] = pcd_foreground

        if not pcd_foreground.has_points():
            return np.array([]), debug_clouds

        # 4. クラスタリング
        centroids, clustered_cloud = self._get_clusters(pcd_foreground)
        debug_clouds['clustered'] = clustered_cloud

        return centroids, debug_clouds

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

    def _get_clusters(self, pcd: o3d.geometry.PointCloud) -> (np.ndarray, o3d.geometry.PointCloud):
        """DBSCANでクラスタリングし、有効なクラスタの重心を計算する。"""
        # DBSCANを実行
        labels = np.array(pcd.cluster_dbscan(
            eps=self._params['cluster_tolerance'],
            min_points=self._params['min_cluster_size'],
            print_progress=False
        ))

        # クラスタごとに色分けされたデバッグ用点群を作成
        max_label = labels.max()
        if max_label < 0:
             return np.array([]), o3d.geometry.PointCloud()

        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0  # ノイズは黒
        
        clustered_cloud = o3d.geometry.PointCloud()
        clustered_cloud.points = pcd.points
        clustered_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

        # 有効なクラスタの重心を計算
        valid_centroids = []
        unique_labels = np.unique(labels)
        for label in unique_labels:
            if label == -1: continue # -1はノイズ

            cluster_indices = np.where(labels == label)[0]
            
            # クラスタサイズのフィルタリング
            if len(cluster_indices) <= self._params['max_cluster_size']:
                cluster_points = np.asarray(pcd.points)[cluster_indices]
                centroid = np.mean(cluster_points, axis=0)
                valid_centroids.append(centroid)
        
        return np.array(valid_centroids), clustered_cloud