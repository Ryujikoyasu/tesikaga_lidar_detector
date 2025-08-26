# お花の池、小鳥の森 - LiDAR検出システム

「お花の池、小鳥の森」アート作品のためのLiDAR人間検出・座標変換システム。LiDARで来場者を検出し、鳥のインタラクションシミュレーションと連携します。

## システム概要

- **LiDAR検出**: 点群データから動く人間を背景差分で抽出
- **座標変換**: 3点キャリブレーションでLiDAR座標系を池中心座標系に変換
- **リアルタイム配信**: UDP経由で人間座標をPythonアプリケーションに送信

## 主要機能

### 1. 人間検出 (`object_cluster_detector_node`)
- 背景差分による動体検出
- クラスタリングによる人間領域抽出
- リアルタイム座標計算・配信

### 2. キャリブレーション (`calibration_node`)
- 既知の3点（ガラス浮き玉位置）での座標計測
- アフィン変換行列とLiDAR姿勢の自動計算
- `transform_matrix.yaml`への結果保存

### 3. 座標変換システム
- LiDAR座標系 → 池中心座標系の変換
- 3点キャリブレーションによる高精度位置合わせ

## クイックスタート

### 1. キャリブレーション実行

```bash
# キャリブレーション用システム起動
cd ~/ros2_ws
source install/setup.bash
ros2 launch tesikaga_lidar_detector trigger_calibration.launch.py

# 別ターミナルでキャリブレーション実行
python src/tesikaga_lidar_detector/scripts/run_calibration.py
```

3点の既知座標に人間が立ち、対話形式で座標を登録。アフィン変換行列が自動計算されます。

### 2. 人間検出システム起動

```bash
# 検出システム起動
ros2 launch tesikaga_lidar_detector detector.launch.py

# 背景キャプチャ（人のいない状態で実行）
ros2 service call /tesikaga_detector/capture_background std_srvs/srv/Empty
```

### 3. 出力確認

検出された人間座標がUDP（127.0.0.1:9999）で配信されます。

## 設定ファイル

- `config/detector_params.yaml`: 検出・キャリブレーションパラメータ
- `config/transform_matrix.yaml`: キャリブレーション結果（自動生成）

## 開発・トラブルシューティング

```bash
# パッケージビルド
colcon build --packages-select tesikaga_lidar_detector

# 設定確認
python check_yaml.py

# ログ確認
ros2 topic echo /tesikaga_detector/output/centroids
```