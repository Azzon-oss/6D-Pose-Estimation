//****欧式聚类分割****//

#include <pcl/io/pcd_io.h>//点云pcd输入输出头文件
#include <pcl/segmentation/extract_clusters.h>//欧式聚类分割头文件
#include <pcl/visualization/cloud_viewer.h>//点云可视化头文件
#include <string>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
//#include <iomanip>
using namespace std;
clock_t start_time, end_time;

int
main(int argc, char** argv)
{
	// 读取点云数据
	pcl::PCDReader reader;//pcd文件读取对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("./bottle2_scene.pcd", *cloud);//读取点云文件
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsize(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.002f, 0.002f, 0.002f);
	vg.filter(*cloud_downsize);
	std::cout << "PointCloud after filtering has: " << cloud_downsize->points.size() << " data points." << std::endl; //*

	start_time = clock();//程序开始计时
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//kd树对象
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//创建欧式聚类分割对象
	ec.setClusterTolerance(0.02); //设置近邻搜索的搜索半径
	ec.setMinClusterSize(100); //设置最小聚类尺寸
	ec.setMaxClusterSize(25000); //设置最大聚类尺寸
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra; //用于储存欧式分割后的点云
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		Eucluextra.push_back(cloud_cluster);
	}
	end_time = clock();//程序结束用时
	double endtime = (double)(end_time - start_time) / CLOCKS_PER_SEC;
	cout << "Total time:" << endtime << "s" << endl;//s为单位
	cout << "Total time:" << endtime * 1000 << "ms" << endl;//ms为单位

	//可视化
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();

	int v1(0);//窗口一
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);
	viewer.addText("Cloud before segmenting", 10, 10, "v1 test", v1);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);

	int v2(0);//窗口二
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
	viewer.addText("Cloud after segmenting", 10, 10, "v2 test", v2);
	for (int i = 0; i < Eucluextra.size(); i++)
	{   std::stringstream ss;
		ss << "cloud_" << i << ".pcd";
		string str_filename = ss.str();
		pcl::io::savePCDFile(str_filename, *Eucluextra[i]);//保存点云
		//显示分割得到的各片点云 
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(Eucluextra[i], 255 * (1 - i)*(2 - i)*(3 - i), 255 * i*(2 - i) * 2 * (0 - i), 255 * i*(i - 1)*(4 - i));
		viewer.addPointCloud(Eucluextra[i], color, str_filename, v2);
	}

	ofstream fout;//文件流
	fout.open("wheat_data.txt");
	int i = 0;
	while (i < cloud->size())
	{
		int index = 0;
		for (int j = 0; j < Eucluextra.size(); j++)
		{
			for (int k = 0; k < Eucluextra[j]->size(); k++)
			{
				if (cloud->points[i].x == Eucluextra[j]->points[k].x&&cloud->points[i].y == Eucluextra[j]->points[k].y&&cloud->points[i].z == Eucluextra[j]->points[k].z)
					index = j;//获取每个点属于分割出的哪个麦粒
			}
		}
		//每个数保存为5位小数
		fout << setiosflags(ios::fixed) << setprecision(5) << cloud->points[i].x << " " 
			<< setiosflags(ios::fixed) << setprecision(5) << cloud->points[i].y << " " 
			<< setiosflags(ios::fixed) << setprecision(5) << cloud->points[i].z << " " 
			<< index << endl;
		i++;
	}
	fout.close();

	//可视化窗口停留
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return (0);
}

