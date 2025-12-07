#include"KeyPointsAnd3DSC.h"

void KeyPointsAnd3DSC::voxelGridAndKeyPoints(PointCloud::Ptr& cloud, PointCloud::Ptr& key_points)
{
	pcl::VoxelGrid< PointT> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(m_step, m_step, m_step);
	PointCloud::Ptr cloud_filtered(new PointCloud);
	vg.filter(*cloud_filtered);

	pcl::ISSKeypoint3D<PointT, PointT> iss_det;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	iss_det.setInputCloud(cloud_filtered);
	iss_det.setSearchMethod(tree);
	iss_det.setSalientRadius(m_SalientRadius);
	iss_det.setNonMaxRadius(m_NonMaxRadius);
	iss_det.setMinNeighbors(4);
	iss_det.setThreshold21(m_delta21);
	iss_det.setThreshold32(m_delta32);
	iss_det.setNumberOfThreads(4);
	iss_det.compute(*key_points);

}

void KeyPointsAnd3DSC::computeKeyPoints3DSC(PointCloud::Ptr& cloud_in, PointCloud::Ptr& key_cloud, pcl::PointCloud<pcl::ShapeContext1980>::Ptr& dsc)
{
	//------------------计算法线----------------------
	pcl::NormalEstimationOMP<PointT, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(6);//设置openMP的线程数
	n.setInputCloud(key_cloud);
	n.setSearchSurface(cloud_in);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	//n.setRadiusSearch(0.03);//半径搜素
	n.compute(*normals);
	std::cout << "法线计算完毕！！！" << std::endl;
	//-------------------计算3dsc-----------------------
	pcl::ShapeContext3DEstimation<PointT, pcl::Normal, pcl::ShapeContext1980> sc;
	sc.setInputCloud(key_cloud);
	sc.setInputNormals(normals);
	//kdTree加速
	sc.setSearchMethod(tree);
	sc.setMinimalRadius(m_MinimalRadius);     // 搜索球面(Rmin)的最小半径值。
	sc.setRadiusSearch(m_RadiusSearch);      // 设置用于确定用于特征估计的最近邻居的球体半径。
	sc.setPointDensityRadius(m_PointDensityRadius);// 这个半径用于计算局部点密度=这个半径内的点数。
	sc.compute(*dsc);
	std::cout << "3DSC特征描述子计算完毕！！！" << std::endl;

}

