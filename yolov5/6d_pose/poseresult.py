import open3d as o3d
import numpy as np

def pose_estimation(target_path, source_path):
    # 加载目标点云（杂乱场景）
    target_cloud = o3d.io.read_point_cloud(target_path)

    # 加载源点云（仅包含物体）
    source_cloud = o3d.io.read_point_cloud(source_path)

    # 对源点云进行下采样
    target_cloud = target_cloud.voxel_down_sample(voxel_size=0.00001)
    # 对源点云进行法线估计
    source_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # 执行ICP算法进行配准
    icp_result = o3d.pipelines.registration.registration_icp(
        source_cloud, target_cloud, 0.02,
        np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))

    # 获取最优位姿变换矩阵
    transformation_matrix = icp_result.transformation

    # 打印位姿估计结果
    print("ICP 结果：")
    print(icp_result)

    # 将源点云根据变换矩阵进行位姿变换
    source_cloud.transform(transformation_matrix)

    # 显示结果
    o3d.visualization.draw_geometries([target_cloud, source_cloud])
# 使用示例
target_path = "./model/parasaurolophus_6700.ply"
source_path = "./scene/rs1_normals.ply"
pose_estimation(target_path, source_path)