#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>

using namespace std;

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{   //---------------------ƴ�ӵ��������뷨����Ϣ-------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP����
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//����kdtree�����н��ڵ㼯����
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//����openMP���߳���
	//n.setViewPoint(0,0,0);//�����ӵ㣬Ĭ��Ϊ��0��0��0��
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);//���Ʒ������ʱ����Ҫ���ѵĽ��ڵ��С
	//n.setRadiusSearch(0.03);//�뾶����
	n.compute(*normals);//��ʼ���з����
	//�����������뷨����Ϣƴ��
	pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}


//�����غ��ʼ���
void calaPointCloudCoincide(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
	core.setInputSource(cloud_src);
	core.setInputTarget(cloud_target);

	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //��������Ȩ������ָ�룬��kdtree������

	core.determineReciprocalCorrespondences(*cor, para1);   //��֮���������,cor��Ӧ����
	std::cout << "����ԭ����������" << cloud_src->size() << std::endl;
	std::cout << "�غϵĵ������� " << cor->size() << std::endl;

	std::cout << "�غ��ʣ� " << double(cor->size()) / double(cloud_src->size()) * 100 << "%" << std::endl;
}

//���ƾ�����������
float caculateRMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_source, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_target)
{
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_source(new pcl::PointCloud<pcl::PointXYZ>());
	fromPCLPointCloud2(*cloud_source, *xyz_source);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_target(new pcl::PointCloud<pcl::PointXYZ>());
	fromPCLPointCloud2(*cloud_target, *xyz_target);*/

	float rmse = 0.0f;

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
	tree->setInputCloud(xyz_target);

	int m = 0;
	for (auto point_i : *xyz_source)
	{
		// ȥ����Ч�ĵ�
		if (!std::isfinite(point_i.x) || !std::isfinite(point_i.y) || !std::isfinite(point_i.z))
			continue;
		pcl::Indices nn_indices(1);
		std::vector<float> nn_distances(1);
		if (!tree->nearestKSearch(point_i, 1, nn_indices, nn_distances)) // K����������ȡƥ����
			continue;
		/*dist�ļ��㷽��֮һ
		size_t point_nn_i = nn_indices.front();
		float dist = squaredEuclideanDistance(point_i, xyz_target->points[point_nn_i]);
		*/
		++m;
		float dist = nn_distances[0]; // ��ȡ����ڶ�Ӧ��֮��ŷ�Ͼ����ƽ��
		rmse += dist;                 // ����ƽ������֮��
	}
	//rmse = std::sqrt(rmse / static_cast<float> (xyz_source->points.size())); // ������������
	rmse = std::sqrt(rmse / m); // ������������

	return rmse;
}


int main()
{
	// --------------------����Դ����-----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile("E:\\XTOP\\XTOPwork\\ICP\\��������\\TargetTransformPnts0.ply", *source);
	cout << "��Դ�����ж�ȡ " << source->size() << " ����" << endl;
	// -------------------����Ŀ�����----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile("E:\\XTOP\\XTOPwork\\ICP\\��������\\TargetTransformPnts1.ply", *target);
	cout << "��Ŀ������ж�ȡ " << target->size() << " ����" << endl;
	//-----------------ƴ�ӵ����뷨����Ϣ-------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(source, source_with_normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(target, target_with_normals);
	//--------------------�㵽���ICP-----------------------
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	/*�㵽��ľ��뺯�����췽��һ*/
	pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr PointToPlane
	(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>);
	/*�㵽��ľ��뺯�����췽����*/
	 //typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> PointToPlane;
	 // boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
	icp.setTransformationEstimation(PointToPlane);
	icp.setInputSource(source_with_normals);
	icp.setInputTarget(target_with_normals);
	icp.setTransformationEpsilon(1e-15);   // Ϊ��ֹ����������Сת������
	icp.setMaxCorrespondenceDistance(0.5);  // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
	icp.setEuclideanFitnessEpsilon(0.0001);  // �������������Ǿ�������С����ֵ�� ֹͣ������
	icp.setMaximumIterations(1);           // ����������
	pcl::PointCloud<pcl::PointNormal>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
	icp.align(*icp_cloud);

	cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
	cout << "�任����\n" << icp.getFinalTransformation() << endl;
	// ʹ�ô����ı任��Ϊ�����Դ���ƽ��б任
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*source, *out_cloud, icp.getFinalTransformation());
	pcl::io::savePLYFileASCII("E:\\XTOP\\XTOPwork\\ICP\\��������\\final_plane.ply", *out_cloud);

	//����������
	float rmse = caculateRMSE(out_cloud, target);
	//std::cout << "��������" << rmse<< "��ֵ��ƽ����������score��ԽСԽ��"<<std::endl;

	//�غ϶ȼ���
	float para1 = 0.1;
	calaPointCloudCoincide(out_cloud, target, para1);

	system("pause");

	return (0);
}
