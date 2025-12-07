
import open3d as o3d

# 读取并显示STL文件
stl_path = "../stl/Scan1.stl"
mesh_stl = o3d.io.read_triangle_mesh(stl_path)
o3d.visualization.draw_geometries([mesh_stl])