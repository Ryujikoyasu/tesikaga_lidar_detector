# tesikaga_lidar_detector/ros_utils.py (改訂版)

import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

def pointcloud2_to_open3d(msg: PointCloud2) -> o3d.geometry.PointCloud:
    """
    sensor_msgs/PointCloud2をOpen3DのPointCloudに変換する（堅牢版）。
    x, y, zフィールドのオフセットを動的に探し、他のフィールドを無視する。
    """
    # フィールド名からオフセットを取得
    offsets = {f.name: f.offset for f in msg.fields}
    if 'x' not in offsets or 'y' not in offsets or 'z' not in offsets:
        raise ValueError("PointCloud2 message must have x, y, and z fields")
    
    x_offset = offsets['x']
    y_offset = offsets['y']
    z_offset = offsets['z']
    
    point_step = msg.point_step
    num_points = msg.width * msg.height
    
    # 必要な座標データだけを抽出
    xyz = np.zeros((num_points, 3), dtype=np.float32)
    
    for i in range(num_points):
        base_idx = i * point_step
        # struct.unpack_fromは、バッファの特定オフセットからデータを読み取る
        xyz[i, 0] = struct.unpack_from('<f', msg.data, base_idx + x_offset)[0]
        xyz[i, 1] = struct.unpack_from('<f', msg.data, base_idx + y_offset)[0]
        xyz[i, 2] = struct.unpack_from('<f', msg.data, base_idx + z_offset)[0]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # 色情報の抽出はオプションとして残す
    if 'rgb' in offsets:
        rgb_offset = offsets['rgb']
        colors = np.zeros((num_points, 3), dtype=np.float32)
        for i in range(num_points):
            base_idx = i * point_step
            # RGBはpacked floatとして扱われることが多い
            rgb_packed = struct.unpack_from('<I', msg.data, base_idx + rgb_offset)[0]
            r = (rgb_packed >> 16) & 0xFF
            g = (rgb_packed >> 8) & 0xFF
            b = (rgb_packed) & 0xFF
            colors[i, 0] = r / 255.0
            colors[i, 1] = g / 255.0
            colors[i, 2] = b / 255.0
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
    return pcd


def open3d_to_pointcloud2(pcd: o3d.geometry.PointCloud, frame_id: str, stamp) -> PointCloud2:
    """ Open3DのPointCloudをsensor_msgs/PointCloud2に変換する (変更なし) """
    header = Header(frame_id=frame_id, stamp=stamp)
    points = np.asarray(pcd.points)
    
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    itemsize = np.dtype(np.float32).itemsize
    
    if pcd.has_colors():
        # 色情報をr,g,bの3つのUINT8フィールドとして追加
        colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
        
        # 4バイトのパディングを追加してアライメントを合わせる
        # [R, G, B, 0]
        padded_colors = np.zeros((len(colors), 4), dtype=np.uint8)
        padded_colors[:,:3] = colors

        # 構造化配列を作成
        point_type = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'u4')])
        cloud_arr = np.empty(len(points), dtype=point_type)
        cloud_arr['x'] = points[:,0]
        cloud_arr['y'] = points[:,1]
        cloud_arr['z'] = points[:,2]
        # RGBを単一のUINT32にパック
        cloud_arr['rgb'] = padded_colors.view('u4').flatten()

        fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
        point_step = itemsize * 4
        data = cloud_arr.tobytes()
    else:
        data = points.astype(np.float32).tobytes()
        point_step = itemsize * 3

    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=point_step,
        row_step=point_step * len(points),
        data=data
    )