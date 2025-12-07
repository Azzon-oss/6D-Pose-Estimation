#pragma once

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/shape_context_3d.h>
#include <Eigen/Core>

// 定义常用的点云类型，方便后续使用
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class Registration
{
public:
    Registration() {
        // 初始化默认参数 (根据您的实验表格调整)
        m_RansacIterations = 5000;  // 粗配准迭代次数
        m_RansacDistance = 0.02f;   // 粗配准内点阈值 (2cm)
        m_MaxIterations = 50;       // ICP最大迭代次数
        m_MaxDistance = 0.01f;      // ICP对应点最大距离 (1cm)
        m_Fraction = 0.25f;         // RANSAC期望的内点比例
    }

    ~Registration() {}

    /**
     * @brief 执行完整的配准流程 (Stage 2: Coarse-to-Fine)
     * @param key_src 源点云的关键点
     * @param sps_src 源点云的3DSC特征
     * @param key_tgt 目标点云的关键点
     * @param sps_tgt 目标点云的3DSC特征
     * @return 最终的 4x4 变换矩阵
     */
    Eigen::Matrix4f RegistrationTransform(
        PointCloud::Ptr& key_src,
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_src,
        PointCloud::Ptr& key_tgt,
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_tgt
    );

    /**
     * @brief 可视化配准结果
     */
    void VisualizeRegistration(PointCloud::Ptr& source, PointCloud::Ptr& target, PointCloud::Ptr& icp);

private:
    // 实现了引入异构空间向量约束的改进 RANSAC
    Eigen::Matrix4f Algorithm2_ImprovedRANSAC(
        PointCloud::Ptr& src,
        PointCloud::Ptr& tgt,
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr& feat_src,
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr& feat_tgt
    );

private:
    int m_RansacIterations;
    float m_RansacDistance;
    float m_MaxDistance;
    int m_MaxIterations;
    float m_Fraction;
};