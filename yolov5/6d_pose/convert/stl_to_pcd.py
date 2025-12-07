import open3d as o3d

def convert_file(input_file_path, output_file_path):
    # 加载输入文件（STL或PLY）
    mesh = o3d.io.read_triangle_mesh(input_file_path)

    # 转换为点云数据
    pcd = mesh.sample_points_uniformly(number_of_points=5000)

    # 保存为PCD文件
    o3d.io.write_point_cloud(output_file_path, pcd)

# 示例使用
#stl_file_path = "./stl/Scan1.stl"
ply_file_path = "../ply/point_cloud3.ply"

pcd_file_path = "../pcd/scene.pcd"

# 转换STL文件为PCD文件
#convert_file(stl_file_path, pcd_file_path)

# 转换PLY文件为PCD文件
convert_file(ply_file_path, pcd_file_path)