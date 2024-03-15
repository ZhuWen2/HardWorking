#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp算法
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>

using namespace std;



double calaPointCloudCoincide(int m, int n,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
	core.setInputSource(cloud_src);
	core.setInputTarget(cloud_target);

	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //共享所有权的智能指针，以kdtree做索引

	core.determineReciprocalCorrespondences(*cor, para1);   //点之间的最大距离,cor对应索引
	double chonghe = double(cor->size()) / double(cloud_src->size());

	if (chonghe < 0.2) {
		return 0;
	}
	//打印出对应点
	string fileName1 = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\对应点\\P" + to_string(m) + "_" + to_string(n) + "_"+ to_string(m) + ".txt";
	string fileName2 = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\对应点\\P" + to_string(m) + "_" + to_string(n) + "_" + to_string(n) + ".txt";
	std::ofstream query_value1;
	query_value1.open(fileName1, std::ios::out);
	std::ofstream query_value2;
	query_value2.open(fileName2, std::ios::out);
	for (int i = 0; i < (*cor).size(); ++i) {
		query_value1 << cloud_src->points[(*cor)[i].index_query].x << " "
			<< cloud_src->points[(*cor)[i].index_query].y << " "
			<< cloud_src->points[(*cor)[i].index_query].z << " "
			<< std::endl;
		
		query_value2 << cloud_target->points[(*cor)[i].index_match].x << " "
			<< cloud_target->points[(*cor)[i].index_match].y << " "
			<< cloud_target->points[(*cor)[i].index_match].z << " "
			<< std::endl;
	}
	query_value1.close();
	query_value2.close();
	return chonghe;
}


void find_corr() {
	for (int i = 0; i < 9; ++i) {
		// --------------------加载源点云-----------------------
		pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
		string fileName1 = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\PT" + to_string(i) + ".ply";
		pcl::io::loadPLYFile(fileName1, *source);
		cout << "从源点云中读取 " << source->size() << " 个点" << endl;
		for (int j = i + 1; j < 10; ++j) {
			// -------------------加载目标点云----------------------
			pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
			string fileName2 = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\PT" + to_string(j) + ".ply";
			pcl::io::loadPLYFile(fileName2, *target);
			cout << "从目标点云中读取 " << target->size() << " 个点" << endl;

			calaPointCloudCoincide(i, j, source, target, 2);

		}
	}
}

int main()
{

	find_corr();
	
	
	system("pause");

	return (0);
}
