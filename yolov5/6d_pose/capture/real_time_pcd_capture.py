import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2
# 创建realsense设备
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
clipping_distance_in_meters=1.0
depth_scale=1.0
# 启动相机
profile = pipeline.start(config)
device = profile.get_device()
device.hardware_reset()
# 指定对齐对象
align_to = rs.stream.color
align = rs.align(align_to)
try:
    for fid in range(20):
        frames = pipeline.wait_for_frames()
        # frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
    while True:
        frames = pipeline.wait_for_frames()
        # 将深度图和颜色图转换为点云
        aligned_frames = align.process(frames)
        profile = aligned_frames.get_profile()
        intrinsics = profile.as_video_stream_profile().get_intrinsics()
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # 转换为open3d中的相机参数
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
        # frames = align.process(frames)
        # 转化数据帧为图像
        key = cv2.waitKey(1)
        if key == ord("a"):
            print("type of depth_image:", type(depth_image))
            print("shape of depth_image:", depth_image.shape)
            img_depth = o3d.geometry.Image(depth_image)
            img_color = o3d.geometry.Image(color_image)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth)# 创建pcd
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            o3d.io.write_point_cloud("a.ply", pcd)
            # 显示点云图
            o3d.visualization.draw_geometries([pcd])
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()