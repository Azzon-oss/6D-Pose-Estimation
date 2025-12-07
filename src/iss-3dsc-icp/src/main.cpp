#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>

// 引入自定义的头文件
#include "Registration.h"
#include "KeyPointsAnd3DSC.h"

int main(int argc, char** argv)
{
    // ---------------------------------------------------------
    // 1. 加载数据 (Load Data)
    // ---------------------------------------------------------
    // 假设在 build 目录下运行，数据在 ../../data/ 目录
    std::string model_path = "../../data/models/bottle1_model.pcd";
    std::string scene_path = "../../data/scenes/bottle1_smooth.pcd"; // 使用平滑后的场景点云

    PointCloud::Ptr source(new PointCloud);
    PointCloud::Ptr target(new PointCloud);

    if (pcl::io::loadPCDFile(model_path, *source) < 0) {
        std::cerr << "Error: Cannot load model file: " << model_path << std::endl;
        return -1;
    }
    if (pcl::io::loadPCDFile(scene_path, *target) < 0) {
        std::cerr << "Error: Cannot load scene file: " << scene_path << std::endl;
        return -1;
    }

    std::cout << "Loaded Model points: " << source->size() << std::endl;
    std::cout << "Loaded Scene points: " << target->size() << std::endl;

    // ---------------------------------------------------------
    // 2. 特征提取 (Feature Extraction: ISS + 3DSC)
    // ---------------------------------------------------------
    KeyPointsAnd3DSC kp_tool;

    // 2.1 提取 ISS 关键点
    PointCloud::Ptr key_src(new PointCloud);
    PointCloud::Ptr key_tgt(new PointCloud);

    // 注意：这里的参数可能需要根据您的模型尺寸微调
    // voxelGridAndKeyPoints 函数内部包含了降采样和ISS提取
    std::cout << "Extracting ISS Keypoints..." << std::endl;
    kp_tool.voxelGridAndKeyPoints(source, key_src);
    kp_tool.voxelGridAndKeyPoints(target, key_tgt);

    std::cout << "Model Keypoints: " << key_src->size() << std::endl;
    std::cout << "Scene Keypoints: " << key_tgt->size() << std::endl;

    // 2.2 计算 3DSC 描述符
    pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_src(new pcl::PointCloud<pcl::ShapeContext1980>);
    pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_tgt(new pcl::PointCloud<pcl::ShapeContext1980>);

    std::cout << "Computing 3DSC Descriptors..." << std::endl;
    // computeKeyPoints3DSC 需要传入原始点云(用于算法线)和关键点
    kp_tool.computeKeyPoints3DSC(source, key_src, sps_src);
    kp_tool.computeKeyPoints3DSC(target, key_tgt, sps_tgt);

    // ---------------------------------------------------------
    // 3. 核心配准 (Registration: Improved RANSAC + ICP)
    // ---------------------------------------------------------
    Registration reg;

    // 调用封装好的配准函数
    Eigen::Matrix4f final_transform;
    final_transform = reg.RegistrationTransform(key_src, sps_src, key_tgt, sps_tgt);

    std::cout << "Final Transformation Matrix:" << std::endl << final_transform << std::endl;

    // ---------------------------------------------------------
    // 4. 结果应用与可视化 (Result & Visualization)
    // ---------------------------------------------------------
    // 将最终变换应用到原始源点云上
    PointCloud::Ptr aligned_source(new PointCloud);
    pcl::transformPointCloud(*source, *aligned_source, final_transform);

    // 保存结果 (可选)
    // pcl::io::savePCDFile("aligned_result.pcd", *aligned_source);

    // 可视化对比
    reg.VisualizeRegistration(source, target, aligned_source);

    return 0;
}