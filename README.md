# 6D-Pose-Estimation
*Official implementation of "Accurate 6D Pose Estimation Using Consumer-Grade Depth Cameras" (Submitted to The Visual Computer).

# Accurate 6D Pose Estimation Using Consumer-Grade Depth Cameras

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![PCL](https://img.shields.io/badge/PCL-1.13.0-green)](https://pointclouds.org/)
[![YOLOv5](https://img.shields.io/badge/YOLO-v5-blue)](https://github.com/ultralytics/yolov5)

This repository contains the official implementation of the paper:
**"Accurate 6D Pose Estimation Using Consumer-Grade Depth Cameras in Real-World Scenarios"**.

<img width="644" height="427" alt="image" src="https://github.com/user-attachments/assets/6acb9a87-44ec-485b-a1fc-87e4d0da7982" />


## 📝 Abstract
Object pose estimation is crucial in augmented reality and robotic manipulation. This paper introduces a novel method for accurate 6D pose estimation using consumer-grade depth cameras (e.g., Realsense D455). Our method consists of two stages:
1.  **Preprocessing**: Target extraction using YOLO and point cloud optimization using MLS smoothing.
2.  **Registration**: Coarse-to-fine registration using ISS feature points, 3DSC descriptors, and an **Improved RANSAC** algorithm with heterogeneous space vectors, followed by ICP refinement.

Note: This requirements.txt is for the Python environment (used by the YOLOv5 detection module and evaluation scripts). The core registration algorithm requires C++ PCL 1.8+, which must be installed as a system library (e.g., via apt-get install libpcl-dev on Ubuntu).

## 📂 6D_pose_Estimation

```text
.
├── data/
│   ├── models/               # CAD models (bottle1_model.pcd, etc.)
│   └── scenes/               # Scene point clouds for testing
├── yolov5/                # Stage 1: Object Detection (YOLOv5)
│   ├── detect.py             # Inference script
│   └── weights/              # Pre-trained weights
├── src/                      # Stage 2: Point Cloud Processing & Registration
│   ├── preprocessing/        # Segmentation & MLS Smoothing
│   ├── iss-3dsc-icp/         # ISS + 3DSC + Improved RANSAC + ICP
│   └── indicators/           # Pose error calculation tools
├── CMakeLists.txt            # CMake configuration
├── requirements.txt          # Python dependencies
└── README.md



##  🔗 Citation
If you find this code useful in your research, please consider citing our paper:
@article{zhang2025accurate,
  title={Accurate 6D Pose Estimation Using Consumer-Grade Depth Cameras in Real-World Scenarios},
  author={Zhang, Zhixiang and Li, Wenhao and Luan, Feng},
  journal={The Visual Computer},
  year={2025}
}
