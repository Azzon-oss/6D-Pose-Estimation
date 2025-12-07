import open3d as o3d
import numpy as np

# 读取待求解物体的点云文件和场景点云文件
object_pcd = o3d.io.read_point_cloud("./model/parasaurolophus_6700.ply")
scene_pcd = o3d.io.read_point_cloud("./scene/rs1_normals.ply")

# 对点云数据进行下采样
object_pcd_downsampled = object_pcd.voxel_down_sample(voxel_size=0.1)  # 调整voxel_size以适应实际数据
scene_pcd_downsampled = scene_pcd.voxel_down_sample(voxel_size=0.01)

# 特征提取
object_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    object_pcd_downsampled, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=100))
scene_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    scene_pcd_downsampled, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=100))

# 特征匹配
correspondences = o3d.pipelines.registration.registration_correspondence(
    object_fpfh, scene_fpfh, 0.9)

# 构造点云对应关系
object_points = np.asarray(object_pcd_downsampled.points)
scene_points = np.asarray(scene_pcd_downsampled.points)
correspondences = np.asarray([[correspondence.source_index, correspondence.target_index] for correspondence in correspondences])

# 初始姿态估计
transformation_init = o3d.pipelines.registration.TransformationEstimationPointToPoint()
ransac_result = o3d.pipelines.registration.registration_ransac_based_on_correspondence(
    object_pcd_downsampled, scene_pcd_downsampled, correspondences,
    transformation_init, o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))

# 姿态优化
transformation_optimized = o3d.pipelines.registration.registration_icp(
    object_pcd_downsampled, scene_pcd_downsampled, 0.05, ransac_result.transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())

# 最终的姿态估计结果
final_transformation = transformation_optimized.transformation

# 可视化展示
object_pcd_downsampled.transform(transformation_optimized)  # 将物体点云根据估计的姿态进行变换

# 创建Visualizer对象并添加物体点云和场景点云
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(object_pcd_downsampled)
vis.add_geometry(scene_pcd_downsampled)

# 更新可视化界面，显示姿态估计的变化
for i in range(100):
    transformation_intermediate = np.eye(4)
    transformation_intermediate[:3, :3] = final_transformation[:3, :3] * i / 100
    object_pcd_downsampled.transform(transformation_intermediate)
    vis.update_geometry(object_pcd_downsampled)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()