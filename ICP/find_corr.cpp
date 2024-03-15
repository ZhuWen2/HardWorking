#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>

using namespace std;



double calaPointCloudCoincide(int m, int n,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
	core.setInputSource(cloud_src);
	core.setInputTarget(cloud_target);

	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //��������Ȩ������ָ�룬��kdtree������

	core.determineReciprocalCorrespondences(*cor, para1);   //��֮���������,cor��Ӧ����
	double chonghe = double(cor->size()) / double(cloud_src->size());

	if (chonghe < 0.2) {
		return 0;
	}
	//��ӡ����Ӧ��
	string fileName1 = "E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\��Ӧ��\\P" + to_string(m) + "_" + to_string(n) + "_"+ to_string(m) + ".txt";
	string fileName2 = "E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\��Ӧ��\\P" + to_string(m) + "_" + to_string(n) + "_" + to_string(n) + ".txt";
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
		// --------------------����Դ����-----------------------
		pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
		string fileName1 = "E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\PT" + to_string(i) + ".ply";
		pcl::io::loadPLYFile(fileName1, *source);
		cout << "��Դ�����ж�ȡ " << source->size() << " ����" << endl;
		for (int j = i + 1; j < 10; ++j) {
			// -------------------����Ŀ�����----------------------
			pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
			string fileName2 = "E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\PT" + to_string(j) + ".ply";
			pcl::io::loadPLYFile(fileName2, *target);
			cout << "��Ŀ������ж�ȡ " << target->size() << " ����" << endl;

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
