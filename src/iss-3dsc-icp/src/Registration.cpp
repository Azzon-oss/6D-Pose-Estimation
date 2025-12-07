#include "Registration.h"
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

// 执行改进的粗配准策略 (基于采样一致性与几何约束)
Eigen::Matrix4f PerformCoarseAlignment(
    PointCloud::Ptr& src,
    PointCloud::Ptr& tgt,
    pcl::PointCloud<pcl::ShapeContext1980>::Ptr& feat_src,
    pcl::PointCloud<pcl::ShapeContext1980>::Ptr& feat_tgt,
    int max_iter,
    float max_dist)
{
    // 设置几何一致性阈值 (对应论文中的异构空间向量约束)
    // 较小的值意味着对多边形边长比的检查更严格
    float similarity_threshold = 0.1f;

    pcl::SampleConsensusPrerejective<PointT, PointT, pcl::ShapeContext1980> align;
    align.setInputSource(src);
    align.setSourceFeatures(feat_src);
    align.setInputTarget(tgt);
    align.setTargetFeatures(feat_tgt);

    // 配置核心参数
    align.setMaximumIterations(max_iter);
    align.setNumberOfSamples(3); // 采样点数
    align.setCorrespondenceRandomness(10); // 特征匹配邻域大小

    // 启用预剔除机制：利用几何相似度筛选错误对应
    align.setSimilarityThreshold(similarity_threshold);

    align.setMaxCorrespondenceDistance(max_dist * 2.0);
    align.setInlierFraction(0.25); // 设定期望的内点比例

    PointCloud::Ptr tmp_cloud(new PointCloud);
    align.align(*tmp_cloud);

    if (align.hasConverged()) {
        std::cout << "[Coarse Registration] Converged. Score: " << align.getFitnessScore() << std::endl;
        return align.getFinalTransformation();
    }
    else {
        std::cout << "[Coarse Registration] Failed to converge." << std::endl;
        return Eigen::Matrix4f::Identity();
    }
}

Eigen::Matrix4f Registration::RegistrationTransform(PointCloud::Ptr& key_src, pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_src,
    PointCloud::Ptr& key_tgt, pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_tgt)
{
    clock_t start = clock();
    pcl::console::print_highlight("Starting Registration Process (Coarse-to-Fine)...\n");

    // ---------------------------------------------------------
    // Stage 1: Coarse Registration (Improved RANSAC)
    // ---------------------------------------------------------
    // 利用改进的RANSAC策略获取初始位姿
    Eigen::Matrix4f ransac_trans = PerformCoarseAlignment(key_src, key_tgt, sps_src, sps_tgt, m_RansacIterations, m_RansacDistance);

    clock_t sac_time = clock();

    // ---------------------------------------------------------
    // Stage 2: Fine Registration (ICP)
    // ---------------------------------------------------------
    PointCloud::Ptr icp_result(new PointCloud);
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(key_src);
    icp.setInputTarget(key_tgt);

    icp.setMaxCorrespondenceDistance(m_MaxDistance);
    icp.setMaximumIterations(m_MaxIterations);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.01);

    // 使用粗配准结果初始化ICP
    icp.align(*icp_result, ransac_trans);

    clock_t end = clock();

    // 输出耗时统计
    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "Total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << std::endl;
    std::cout << "Coarse time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << " s" << std::endl;
    std::cout << "Fine time: " << (double)(end - sac_time) / (double)CLOCKS_PER_SEC << " s" << std::endl;
    std::cout << "ICP converged: " << (icp.hasConverged() ? "Yes" : "No") << " | Fitness Score: " << icp.getFitnessScore() << std::endl;
    std::cout << "------------------------------------------------" << std::endl;

    return icp.getFinalTransformation();
}

void Registration::VisualizeRegistration(PointCloud::Ptr& source, PointCloud::Ptr& target, PointCloud::Ptr& icp)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration Viewer"));

    int v1 = 0;
    int v2 = 1;
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);

    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, v2);

    // 原始源点云（绿色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
    // 目标点云（蓝色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);
    // 配准后源点云（红色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(icp, 255, 0, 0);

    // 视口1：显示配准前的状态
    viewer->addPointCloud(source, src_h, "source_cloud_v1", v1);
    viewer->addPointCloud(target, tgt_h, "target_cloud_v1", v1);
    viewer->addText("Initial State", 10, 10, "v1_text", v1);

    // 视口2：显示配准后的状态
    viewer->addPointCloud(target, tgt_h, "target_cloud_v2", v2);
    viewer->addPointCloud(icp, transe, "aligned_cloud_v2", v2);
    viewer->addText("Aligned State", 10, 10, "v2_text", v2);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}