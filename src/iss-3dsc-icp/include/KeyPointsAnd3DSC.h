#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/3dsc.h>
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class KeyPointsAnd3DSC 
{
	float m_step;              // 体素滤波格网边长
    float m_SalientRadius;     // ISS 搜索半径
    float m_NonMaxRadius;      // ISS 非极大值抑制半径
    float m_delta21;           // ISS L2/L1的阈值
    float m_delta32;           // ISS l3/L2的阈值
    float m_MinimalRadius;     // 3DSC 球面最小半径
    float m_RadiusSearch;      // 3DSC 邻域点半径
    float m_PointDensityRadius;// 3DSC 密度计算阈值

public:
    KeyPointsAnd3DSC(float step = 0.001, float SalientRadius = 0.005, 
        float  NonMaxRadius = 0.01, float delta21 = 0.65,
        float delta32 = 0.5, float MinimalRadius=0.02, 
        float RadiusSearch=0.03, float PointDensityRadius=0.02) :
        m_step (step), m_SalientRadius(SalientRadius),
        m_NonMaxRadius(NonMaxRadius), m_delta21(delta21),
        m_delta32(delta32), m_MinimalRadius(MinimalRadius), 
        m_RadiusSearch(RadiusSearch), m_PointDensityRadius(PointDensityRadius){}
    ~KeyPointsAnd3DSC() {}

    void voxelGridAndKeyPoints(PointCloud::Ptr& cloud, PointCloud::Ptr& key_points);
    void computeKeyPoints3DSC(PointCloud::Ptr& cloud_in, PointCloud::Ptr& key_cloud, pcl::PointCloud<pcl::ShapeContext1980>::Ptr& dsc);
};


