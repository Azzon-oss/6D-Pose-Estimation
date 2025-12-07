import open3d as o3d

def visualize_point_cloud(pcd_file_path):
    # 加载点云文件并创建点云对象
    pcd = o3d.io.read_point_cloud(pcd_file_path)

    # 可视化点云
    o3d.visualization.draw_geometries([pcd])

# 示例使用
pcd_file_path = "../pcd/bottle3_real_sence.pcd"
visualize_point_cloud(pcd_file_path)