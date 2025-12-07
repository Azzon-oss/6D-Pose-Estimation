#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/median_filter.h>

using namespace std;
clock_t start_time, end_time;

int main(int argc, char** argv)
{
	cout << "smooth" << endl;

	//读入pcd或者ply点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile<pcl::PointXYZ>("bottle2_pure.pcd", *cloud);
	//pcl::io::loadPLYFile("yuanpan.ply", *cloud);

	//Set up the KDTree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
        start_time = clock();//程序开始计时
	//最小二乘法(MLS)点云平滑参数设置
	pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	mls.setComputeNormals(false);
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(false);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(100);
	mls.process(*mls_points);

	//gaussian
        //Set up the Gaussian Kernel
	//pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
	//(*kernel).setSigma(4);
	//(*kernel).setThresholdRelativeToSigma(4);
	////Set up the Convolution Filter
	//pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_points(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::filters::Convolution3D <pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> > convolution;
	//convolution.setKernel(*kernel);
	//convolution.setInputCloud(cloud);
	//convolution.setSearchMethod(tree);
	//convolution.setRadiusSearch(3);
	//convolution.setNumberOfThreads(10);//important! Set Thread number for openMP
	//std::cout << "Convolution Start" << std::endl;
	//convolution.convolve(*gaussian_points);
	//std::cout << "Convoluted" << std::endl;

	//passthrough
        //pcl::PointCloud<pcl::PointXYZ>::Ptr pass_points(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PassThrough<pcl::PointXYZ> pass;  //设置滤波器对象
	//pass.setInputCloud(cloud);  //设置输入点云
	//pass.setFilterFieldName("z");  //设置过滤是所需要的点云类型的y字段
	//pass.setFilterLimits(0, 0.9);  //设置在过滤字段上的范围
	//pass.filter(*pass_points);

	//MedianFilter
	//pcl::PointCloud<pcl::PointXYZ>::Ptr median_points(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::MedianFilter<pcl::PointXYZ> median_filter;
	//median_filter.setInputCloud(cloud);
	//median_filter.setWindowSize(3);
	//median_filter.filter(*median_points);

	pcl::visualization::PCLVisualizer viewer("3d");
	viewer.addPointCloud(cloud, "cloud");
	pcl::visualization::PCLVisualizer viewer2("3d2");
	viewer2.addPointCloud(mls_points, "mls_points");
        end_time = clock();//程序结束用时
	double endtime = (double)(end_time - start_time) / CLOCKS_PER_SEC;
	cout << "Total time:" << endtime << "s" << endl;//s为单位
	cout << "Total time:" << endtime * 1000 << "ms" << endl;//ms为单位

	//pcl::io::savePLYFile("yuanpan_smooth.ply", *mls_points);
	pcl::io::savePCDFile("bottle2_pure_smooth.pcd", *mls_points);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		viewer2.spinOnce();
	}

	return 0;
}
