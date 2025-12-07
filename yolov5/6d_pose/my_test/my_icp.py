import numpy as np
import math
import open3d as o3d


target_orig_pcd = o3d.io.read_point_cloud("../ply/target.ply")  # 原模型的采样点数过多，需要降采样
scene_orign_pcd = o3d.io.read_point_cloud("./adjust_scene.ply")  # 读入经过yolo筛选的场景点云
#可视化查看二者
o3d.visualization.draw_geometries([target_orig_pcd])
o3d.visualization.draw_geometries([scene_orign_pcd])
# 下采样
num_points = 50000 # 隔多少个点采样,越大采样点越多
every_k_points = len(target_orig_pcd.points) // num_points
downsampled_pc = target_orig_pcd.uniform_down_sample(every_k_points=every_k_points)  # 根据需要设置体素大
#o3d.visualization.draw_geometries([downsampled_pc])

o3d.visualization.draw_geometries([scene_orign_pcd + target_orig_pcd])

