"""
修改PLY文件的标签名称和值类型
"""
from helper_ply import read_ply, write_ply
import numpy as np

data = read_ply(r'D:\dev\室内点云分割\CODE\area3\Area_3_sub.ply')
points = np.vstack((data['x'], data['y'], data['z'])).T
rgb = np.vstack((data['red'], data['green'], data['blue'])).T
# label = data['label'].astype(np.float32)
# label = data['scalar_Scalar_field'].astype(np.uint32)
# label = data['scalar_Label'].astype(np.float32)
# label = data['scalar_Label'].astype(np.float32)
label = data['scalar_class'].astype(np.uint32)


write_ply('Area_3_sub_label.ply', [points, rgb, label], ['x', 'y', 'z', 'red', 'green', 'blue', 'label'])
# write_ply('Area_4.ply', [points, rgb, label], ['x', 'y', 'z', 'red', 'green', 'blue', 'scalar_class'])