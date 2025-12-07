import numpy as np
import open3d as o3d

def scale_adjustment(scene_pc, target_pc):
    scene_center = np.mean(scene_pc.points, axis=0)
    target_center = np.mean(target_pc.points, axis=0)

    scene_dist = np.mean(np.linalg.norm(scene_pc.points - scene_center, axis=1))
    target_dist = np.mean(np.linalg.norm(target_pc.points - target_center, axis=1))

    scale = target_dist / scene_dist
    print(scale)

    scaled_scene_pc = o3d.geometry.PointCloud()
    scaled_scene_pc.points = o3d.utility.Vector3dVector((scene_pc.points - scene_center) * scale + target_center)

    return scaled_scene_pc

# 读取场景点云和目标点云
scene_pc = o3d.io.read_point_cloud("../scene/bottle3.ply")
target_pc = o3d.io.read_point_cloud("../ply/target.ply")
# 下采样
num_points = 50000 # 各多少个点采样
every_k_points = len(target_pc.points) // num_points

downsampled_pc = target_pc.uniform_down_sample(every_k_points=every_k_points)  # 根据需要设置体素大小

# 保存下采样后的点云
o3d.io.write_point_cloud("../ply/target_downsampled.ply", downsampled_pc)
o3d.visualization.draw_geometries([downsampled_pc])
o3d.visualization.draw_geometries([scene_pc + target_pc])
# 进行尺度调整
scaled_scene_pc = scale_adjustment(scene_pc, target_pc)
o3d.io.write_point_cloud("adjust_scene.ply", scaled_scene_pc)
o3d.visualization.draw_geometries([scaled_scene_pc + downsampled_pc])
