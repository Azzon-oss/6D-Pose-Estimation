import time
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import open3d as o3d

# 初始化 Realsense 相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)


def get_aligned_images():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    aligned_color_frame = aligned_frames.get_color_frame()

    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics

    img_color = np.asanyarray(aligned_color_frame.get_data())
    img_depth = np.asanyarray(aligned_depth_frame.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(img_depth, alpha=0.008), cv2.COLORMAP_JET)

    return depth_intrin, img_color, img_depth, aligned_depth_frame


class YOLOv8Detector:
    def __init__(self, model_path='../recycle_best.pt'):
        self.model = YOLO(model_path)
        self.class_names = self.model.names

    def detect(self, img):
        results = self.model(img)
        return results


def generate_point_cloud(depth_image, color_image, depth_intrin):
    depth_image = np.ascontiguousarray(depth_image)
    color_image = np.ascontiguousarray(color_image)

    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        depth_intrin.width, depth_intrin.height, depth_intrin.fx, depth_intrin.fy, depth_intrin.ppx, depth_intrin.ppy)

    img_depth = o3d.geometry.Image(depth_image)
    img_color = o3d.geometry.Image(color_image)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd


def main():
    detector = YOLOv8Detector()
    try:
        while True:
            depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()

            results = detector.detect(img_color)
            result = results[0]
            boxes = result.boxes.xyxy.tolist()
            im_array = result.plot()

            camera_xyz_list = []
            for i in range(len(boxes)):
                x1, y1, x2, y2 = map(int, boxes[i])
                ux = (x1 + x2) // 2
                uy = (y1 + y2) // 2
                dis = aligned_depth_frame.get_distance(ux, uy)
                camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, (ux, uy), dis)
                camera_xyz = np.round(np.array(camera_xyz), 3) * 1000

                cv2.circle(im_array, (ux, uy), 4, (255, 255, 255), 5)
                cv2.putText(im_array, f'XYZ: {camera_xyz.tolist()}', (ux + 20, uy + 10), 0, 0.5, [225, 255, 255],
                            thickness=1, lineType=cv2.LINE_AA)
                camera_xyz_list.append(camera_xyz)

            cv2.imshow('detection', im_array)

            key = cv2.waitKey(1)
            if key == ord("a") and camera_xyz_list:
                for i in range(len(boxes)):
                    x1, y1, x2, y2 = map(int, boxes[i])
                    depth_roi = img_depth[y1:y2, x1:x2]
                    color_roi = img_color[y1:y2, x1:x2]
                    pcd = generate_point_cloud(depth_roi, color_roi, depth_intrin)
                    o3d.io.write_point_cloud("target.ply", pcd)
                    o3d.visualization.draw_geometries([pcd])

            if key & 0xFF == ord('q') or key == 27:
                break
    finally:
        pipeline.stop()


if __name__ == '__main__':
    main()