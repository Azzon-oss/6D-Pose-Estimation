#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>     // fpfh加速计算的omp(多核并行计算)
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/registration/ia_ransac.h>//采样一致性
#include <pcl/registration/icp.h>      //icp配准
#include <pcl/visualization/pcl_visualizer.h> 
#include <boost/thread/thread.hpp>
using namespace std;
#pragma region
/*
这段代码是一个函数，名为extract_keypoint，
它接受一个指向pcl::PointCloud<pcl::PointXYZ>类型的指针cloud和一个指向pcl::PointCloud<pcl::PointXYZ>类型的指针keypoint作为输入参数。
函数还有三个可选参数：LeafSize表示体素格网的大小，默认值为0.02；radius表示法向量计算的半径，默认值为0.04；threshold表示夹角阈值（以度为单位），默认值为5。
函数的主要功能是从输入的点云数据中提取关键点。首先，函数使用体素下采样方法对输入的点云进行降采样，将结果保存在名为pcd_down的新点云中。
然后，函数计算每个点的法向量，并将结果保存在名为normals的新点云中。
接下来，函数使用KD树来计算每个点的邻域内的点的法向量夹角，并计算夹角的平均值。如果平均夹角大于夹角阈值，则将该点索引添加到名为indexes的向量中。
最后，函数使用pcl::copyPointCloud函数根据indexes向量提取关键点，并将结果保存在keypoint点云中。函数还输出提取的特征点的数量。
总之，这段代码实现了从点云数据中提取关键点的功能。
*/
void extract_keypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoint,
	float LeafSize = 0.02, float radius = 0.04, float threshold = 5) // 参数分别为：体素格网的大小，法向量计算半径，夹角阈值（度）
{
	//体素下采样
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_down(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_grid.setInputCloud(cloud);
	voxel_grid.setLeafSize(LeafSize, LeafSize, LeafSize);
	voxel_grid.filter(*pcd_down);
	//计算每一个点的法向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(pcd_down);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	n.setSearchMethod(tree);
	//设置KD树搜索半径
	// n.setRadiusSearch (0.03);
	n.setKSearch(10);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);

	float Angle = 0.0;
	float Average_Sum_AngleK = 0.0;//定义邻域内K个点法向量夹角的平均值
	vector<int>indexes;
	//--------------计算法向量夹角及夹角均值----------------
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //建立kdtree对象
	kdtree.setInputCloud(pcd_down); //设置需要建立kdtree的点云指针

	vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
	vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
	pcl::PointXYZ searchPoint;
	for (size_t i = 0; i < pcd_down->points.size(); ++i) {
		searchPoint = pcd_down->points[i];
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			float Sum_AngleK = 0.0;//定义K个邻近的点法向夹角之和
	/*计算法向量的夹角*/
			for (size_t m = 0; m < pointIdxRadiusSearch.size(); ++m) {

				Eigen::Vector3f
					v1(normals->points[i].data_n[0],
						normals->points[i].data_n[1],
						normals->points[i].data_n[2]),

					v2(normals->points[pointIdxRadiusSearch[m]].data_n[0],
					   normals->points[pointIdxRadiusSearch[m]].data_n[1],
					   normals->points[pointIdxRadiusSearch[m]].data_n[2]);

				Angle = pcl::getAngle3D(v1, v2, true);

			}
			Sum_AngleK += Angle;//邻域夹角之和
			Average_Sum_AngleK = Sum_AngleK / pointIdxRadiusSearch.size();//邻域夹角均值
	//-----------------提取特征点--------------------
			float t = pcl::deg2rad(threshold);
			if (Average_Sum_AngleK > t) {
				indexes.push_back(i);
			}
		}
	}
	pcl::copyPointCloud(*pcd_down, indexes, *keypoint);

	cout << "提取的特征点个数:" << keypoint->points.size() << endl;

};

pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoint)
{
	// 法向量估计
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(keypoint);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	n.compute(*normals);
	//------------------FPFH估计-------------------------
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
	f.setNumberOfThreads(8); //指定8核计算
	f.setInputCloud(keypoint);
	f.setInputNormals(normals);
	f.setSearchMethod(tree);
	f.setRadiusSearch(0.05);
	f.compute(*fpfh);

	return fpfh;

}

// 可视化
void  visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& regist)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration"));
	int v1 = 0;
	int v2 = 1;
	viewer->setWindowName("SAC-IA+NDT配准结果");
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.05, 0, 0, v2);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
	viewer->addText("Registed point clouds", 10, 10, "v2_text", v2);
	//原始点云绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
	//目标点云蓝色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);
	//转换后的源点云红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(regist, 255, 0, 0);
	//viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(source, src_h, "source cloud", v1);
	viewer->addPointCloud(target, tgt_h, "target cloud", v1);

	viewer->addPointCloud(target, tgt_h, "target cloud1", v2);
	viewer->addPointCloud(regist, transe, "pcs cloud", v2);

	//添加坐标系
	//viewer->addCoordinateSystem(0.1);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}

}
//由旋转平移矩阵计算旋转角度
void matrix2angle (Eigen::Matrix4f &result_trans,Eigen::Vector3f &result_angle)
{
  double ax,ay,az;
  if (result_trans(2,0)==1 || result_trans(2,0)==-1)
  {
      az=0;
      double dlta;
      dlta=atan2(result_trans(0,1),result_trans(0,2));
      if (result_trans(2,0)==-1)
      {
          ay=M_PI/2;
          ax=az+dlta;
      }
      else
      {
          ay=-M_PI/2;
          ax=-az+dlta;
      }
  }
  else
  {
      ay=-asin(result_trans(2,0));
      ax=atan2(result_trans(2,1)/cos(ay),result_trans(2,2)/cos(ay));
      az=atan2(result_trans(1,0)/cos(ay),result_trans(0,0)/cos(ay));
  }
  result_angle<<ax,ay,az;
}
#pragma endregion
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("../../6d_data/Stanford/bunny/bunny.pcd", *source);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("../../6d_data/Stanford/bunny/bunny_trans.pcd", *target);
	if (source->empty() || target->empty()) // 使用empty()函数判断点云是否加载成功
	{
		cout << "请确认点云文件名称是否正确" << endl;
		return -1;
	}
	else
	{
		cout << "从目标点云读取 " << target->size() << " 个点" << endl;
		cout << "从源点云中读取 " << source->size() << " 个点" << endl;
	}

	//1、 提取特征点
	pcl::PointCloud<pcl::PointXYZ>::Ptr s_k(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr t_k(new pcl::PointCloud<pcl::PointXYZ>);
	extract_keypoint(source,s_k);
	extract_keypoint(target,t_k);
	//2、计算源点云和目标点云的FPFH
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr sk_fpfh = compute_fpfh_feature(s_k);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr tk_fpfh = compute_fpfh_feature(t_k);

	//3、SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(s_k);
	scia.setInputTarget(t_k);
	scia.setSourceFeatures(sk_fpfh);
	scia.setTargetFeatures(tk_fpfh);
	scia.setMinSampleDistance(0.007);
	scia.setNumberOfSamples(100);
	scia.setCorrespondenceRandomness(6);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sac_result(new pcl::PointCloud<pcl::PointXYZ>);
	scia.align(*sac_result);
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	std::cout << sac_trans << endl;
	//4、KD树改进的ICP配准
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//kdTree 加速搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(s_k);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(t_k);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	icp.setInputSource(s_k);
	icp.setInputTarget(t_k);
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setMaximumIterations(35);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.01);
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*icp_result, sac_trans);
	std::cout << "ICP has converged:" << icp.hasConverged()
       << " score: " << icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f icp_trans;
    icp_trans=icp.getFinalTransformation();
    //cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
    std::cout<<icp_trans<<endl;
    //使用创建的变换对未过滤的输入点云进行变换
	//计算误差
    Eigen::Vector3f ANGLE_origin;
    ANGLE_origin<<0,0,M_PI/5;
    double error_x,error_y,error_z;
    Eigen::Vector3f ANGLE_result;
    matrix2angle(icp_trans,ANGLE_result);
    error_x=fabs(ANGLE_result(0))-fabs(ANGLE_origin(0));
    error_y=fabs(ANGLE_result(1))-fabs(ANGLE_origin(1));
    error_z=fabs(ANGLE_result(2))-fabs(ANGLE_origin(2));
    cout<<"original angle in x y z:\n"<<ANGLE_origin<<endl;
    cout<<"error in aixs_x: "<<error_x<<"  error in aixs_y: "<<error_y<<"  error in aixs_z: "<<error_z<<endl;
	pcl::transformPointCloud(*source, *icp_result, icp.getFinalTransformation());
	// 5、可视化
	visualize_registration(source, target, icp_result);
	return 0;
}


