import open3d as o3d

# 读取并显示PLY文件
ply_path = "../scene/bottle2_scene_1.ply"
pcd_ply = o3d.io.read_point_cloud(ply_path)
o3d.visualization.draw_geometries([pcd_ply])

