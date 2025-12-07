import open3d as o3d

def convert_stl_to_ply(stl_file, ply_file):
    # 读取STL文件
    mesh = o3d.io.read_triangle_mesh(stl_file)

    # 检查是否成功读取STL文件
    if not mesh.has_triangle_normals():
        mesh.compute_vertex_normals()

    # 保存为PLY文件
    o3d.io.write_triangle_mesh(ply_file, mesh)

# 示例用法
stl_file = "../stl/Scan1.stl"
ply_file = "../ply/target.ply"
convert_stl_to_ply(stl_file, ply_file)