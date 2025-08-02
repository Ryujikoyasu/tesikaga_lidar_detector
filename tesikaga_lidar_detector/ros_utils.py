import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# sensor_msgs/PointFieldのデータ型とnumpy/pythonの型の対応辞書
DUMMY_FIELD_PREFIX = '__'
DATATYPE_TO_NUMPY_STRUCT = {
    PointField.INT8: np.int8, PointField.UINT8: np.uint8,
    PointField.INT16: np.int16, PointField.UINT16: np.uint16,
    PointField.INT32: np.int32, PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64
}

def pointcloud2_to_open3d(msg: PointCloud2) -> o3d.geometry.PointCloud:
    """ sensor_msgs/PointCloud2をOpen3DのPointCloudに変換する """
    # データ型とフィールド名を取得
    fields = msg.fields
    dtype_list = []
    for field in fields:
        if field.name == 'rgb':
            # RGBは特殊ケースとしてfloat32で扱う
            dtype_list.append((field.name, np.float32))
        else:
            dtype_list.append((field.name, DATATYPE_TO_NUMPY_STRUCT[field.datatype]))
    
    # NumPyの構造化配列としてデータを読み込み
    cloud_arr = np.frombuffer(msg.data, dtype=np.dtype(dtype_list))

    # xyz座標のみを抽出
    xyz = np.vstack([cloud_arr['x'], cloud_arr['y'], cloud_arr['z']]).T
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # 色情報があれば追加
    if 'rgb' in [field.name for field in fields]:
        # RGBデータをパース
        rgb_float = cloud_arr['rgb']
        r = np.floor(rgb_float / 65536) % 256
        g = np.floor(rgb_float / 256) % 256
        b = np.floor(rgb_float) % 256
        colors = np.vstack([r, g, b]).T / 255.0
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
    return pcd

def open3d_to_pointcloud2(pcd: o3d.geometry.PointCloud, frame_id: str, stamp) -> PointCloud2:
    """ Open3DのPointCloudをsensor_msgs/PointCloud2に変換する """
    header = Header(frame_id=frame_id, stamp=stamp)
    points = np.asarray(pcd.points)
    
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    itemsize = np.dtype(np.float32).itemsize
    
    if pcd.has_colors():
        colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
        data = np.hstack([points.astype(np.float32), colors]).flatten().tobytes()
        fields.append(PointField(name='r', offset=12, datatype=PointField.UINT8, count=1))
        fields.append(PointField(name='g', offset=13, datatype=PointField.UINT8, count=1))
        fields.append(PointField(name='b', offset=14, datatype=PointField.UINT8, count=1))
        point_step = itemsize * 3 + 3
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