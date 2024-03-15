#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp算法
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>

using namespace std;

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{   //---------------------拼接点云数据与法线信息-------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//设置openMP的线程数
	//n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	//n.setRadiusSearch(0.03);//半径搜素
	n.compute(*normals);//开始进行法向计
	//将点云数据与法向信息拼接
	pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}


//点云重合率计算
void calaPointCloudCoincide(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
	core.setInputSource(cloud_src);
	core.setInputTarget(cloud_target);

	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //共享所有权的智能指针，以kdtree做索引

	core.determineReciprocalCorrespondences(*cor, para1);   //点之间的最大距离,cor对应索引
	std::cout << "点云原来的数量：" << cloud_src->size() << std::endl;
	std::cout << "重合的点云数： " << cor->size() << std::endl;

	std::cout << "重合率： " << double(cor->size()) / double(cloud_src->size()) * 100 << "%" << std::endl;
}

//点云均方根误差计算
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
		// 去除无效的点
		if (!std::isfinite(point_i.x) || !std::isfinite(point_i.y) || !std::isfinite(point_i.z))
			continue;
		pcl::Indices nn_indices(1);
		std::vector<float> nn_distances(1);
		if (!tree->nearestKSearch(point_i, 1, nn_indices, nn_distances)) // K近邻搜索获取匹配点对
			continue;
		/*dist的计算方法之一
		size_t point_nn_i = nn_indices.front();
		float dist = squaredEuclideanDistance(point_i, xyz_target->points[point_nn_i]);
		*/
		++m;
		float dist = nn_distances[0]; // 获取最近邻对应点之间欧氏距离的平方
		rmse += dist;                 // 计算平方距离之和
	}
	//rmse = std::sqrt(rmse / static_cast<float> (xyz_source->points.size())); // 计算均方根误差
	rmse = std::sqrt(rmse / m); // 计算均方根误差

	return rmse;
}


int main()
{
	// --------------------加载源点云-----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\TargetTransformPnts0.ply", *source);
	cout << "从源点云中读取 " << source->size() << " 个点" << endl;
	// -------------------加载目标点云----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\TargetTransformPnts1.ply", *target);
	cout << "从目标点云中读取 " << target->size() << " 个点" << endl;
	//-----------------拼接点云与法线信息-------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(source, source_with_normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(target, target_with_normals);
	//--------------------点到面的ICP-----------------------
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	/*点到面的距离函数构造方法一*/
	pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr PointToPlane
	(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>);
	/*点到面的距离函数构造方法二*/
	 //typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> PointToPlane;
	 // boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
	icp.setTransformationEstimation(PointToPlane);
	icp.setInputSource(source_with_normals);
	icp.setInputTarget(target_with_normals);
	icp.setTransformationEpsilon(1e-15);   // 为终止条件设置最小转换差异
	icp.setMaxCorrespondenceDistance(0.5);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	icp.setEuclideanFitnessEpsilon(0.0001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
	icp.setMaximumIterations(1);           // 最大迭代次数
	pcl::PointCloud<pcl::PointNormal>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
	icp.align(*icp_cloud);

	cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
	cout << "变换矩阵：\n" << icp.getFinalTransformation() << endl;
	// 使用创建的变换对为输入的源点云进行变换
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*source, *out_cloud, icp.getFinalTransformation());
	pcl::io::savePLYFileASCII("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\final_plane.ply", *out_cloud);

	//均方根计算
	float rmse = caculateRMSE(out_cloud, target);
	//std::cout << "均方根误差：" << rmse<< "此值的平方就是上面score，越小越好"<<std::endl;

	//重合度计算
	float para1 = 0.1;
	calaPointCloudCoincide(out_cloud, target, para1);

	system("pause");

	return (0);
}
