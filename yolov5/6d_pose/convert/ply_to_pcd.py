import open3d as o3d

def convert_ply_to_pcd(ply_file, pcd_file):
    # 读取PLY文件
    mesh = o3d.io.read_triangle_mesh(ply_file)

    # 将网格转换为点云
    point_cloud = mesh.sample_points_uniformly(number_of_points=100000)

    # 保存为PCD文件
    o3d.io.write_point_cloud(pcd_file, point_cloud)

# 示例用法
ply_file = "../ply/newscene5.ply"
pcd_file = "../pcd/scene5.pcd"
convert_ply_to_pcd(ply_file, pcd_file)