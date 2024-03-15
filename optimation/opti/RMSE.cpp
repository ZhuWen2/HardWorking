#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include<cmath>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <filesystem>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>
using namespace std;
int main()
{
	double rmsehh = 0;

	//����ԭʼ��������
	for (int i = 0; i < 1; ++i) {
		// -------------------����Ŀ�����----------------------
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
		std::string fileName1 = "C:\\Users\\123456\\Desktop\\allMarkPoints_aft.ply";
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
		std::string fileName2 = "C:\\Users\\123456\\Desktop\\glo.ply";
		pcl::io::loadPLYFile(fileName1, *cloudA);
		std::cout << "��Ŀ������ж�ȡ " << cloudA->size() << " ����" << std::endl;
		pcl::io::loadPLYFile(fileName2, *cloudB);
		std::cout << "��Ŀ������ж�ȡ " << cloudB->size() << " ����" << std::endl;


		pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
		core.setInputSource(cloudA);
		core.setInputTarget(cloudB);
		pcl::Correspondences all;
		//core.determineCorrespondences(all_correspondences,0.05);//ȷ�����������Ŀ�����֮��Ķ�Ӧ��ϵ��

		core.determineReciprocalCorrespondences(all, 0.5);    //ȷ�����������Ŀ�����֮��Ľ�����Ӧ��ϵ��
		double sum = 0.0, sum_x = 0.0, sum_y = 0.0, sum_z = 0.0, rmse, rmse_x, rmse_y, rmse_z;
		vector<float>Co;
		for (size_t j = 0; j < all.size(); j++) {
			sum += all[j].distance;
			Co.push_back(all[j].distance);
			sum_x += pow((cloudB->points[all[j].index_match].x - cloudA->points[all[j].index_query].x), 2);
			sum_y += pow((cloudB->points[all[j].index_match].y - cloudA->points[all[j].index_query].y), 2);
			sum_z += pow((cloudB->points[all[j].index_match].z - cloudA->points[all[j].index_query].z), 2);
		}
		rmse = sqrt(sum / all.size());     //���������
		rmsehh += rmse;
		rmse_x = sqrt(sum_x / all.size()); //X������������
		rmse_y = sqrt(sum_y / all.size()); //Y������������
		rmse_z = sqrt(sum_z / all.size()); //Z������������
		vector<float>::iterator max = max_element(Co.begin(), Co.end());//��ȡ������Ķ�Ӧ��
		vector<float>::iterator min = min_element(Co.begin(), Co.end());//��ȡ��С����Ķ�Ӧ��
		cout << "ƥ���Ը���" << all.size() << endl;
		cout << "�������ֵ" << sqrt(*max) << "����" << endl;
		cout << "������Сֵ" << sqrt(*min)<< "����" << endl;

		cout << "���������" << rmse << "��" << endl;
		cout << "X���������" << rmse_x << "��" << endl;
		cout << "Y���������" << rmse_y << "��" << endl;
		cout << "Z���������" << rmse_z << "��" << endl;
	}

	rmsehh /= 1;
	cout << "���������" << rmsehh << "mm" << endl;
	return 0;
}
