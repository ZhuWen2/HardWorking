#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件
#include<pcl/search/impl/kdtree.hpp>
#include <pcl/io/ply_io.h>

int main(int argc, char** argv)
{
	std::string fileName1;
	std::string fileName2;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	// Output has the PointNormal type in order to store the normals calculated by MLS

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	fileName1 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\multiway_registration1.ply";
	fileName2 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\multiway_registration1_rong.ply";
	pcl::io::loadPLYFile(fileName1, *cloud);
	for (int i = 1; i < 2; ++i) {
		/*fileName1 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\降采样工件" + std::to_string(i) + ".ply";
		fileName2 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\a降采样工件" + std::to_string(i) + ".ply";*/
		
		pcl::io::loadPLYFile(fileName1, *cloud);
		

		std::cout << cloud->size() << std::endl;
		
		// 定义最小二乘实现的对象mls
		

		mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

		// Set parameters
		mls.setInputCloud(cloud);
		
		mls.setPolynomialOrder(5); //设置false，可以加速平滑
		mls.setSearchMethod(tree);
		mls.setSearchRadius(4); //单位m，设置用于拟合的K近邻半径
		//mls.setNumberOfThreads(4);
		// Reconstruct
		//cloud->clear();
		mls.process(*cloud_out); //输出
		//std::cout << "sss" << std::endl;





		//// 创建 KD-Tree
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		//// Output has the PointNormal type in order to store the normals calculated by MLS
		//pcl::PointCloud<pcl::PointNormal> mls_points;

		//// 定义最小二乘实现的对象mls
		//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

		//mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

		//// Set parameters
		//mls.setInputCloud(cloud);
		//mls.setPolynomialOrder(3); //设置false，可以加速平滑
		//mls.setSearchMethod(tree);
		//mls.setSearchRadius(5); //
		//mls.setNumberOfThreads(4);
		//// Reconstruct
		//mls.process(mls_points); //输出

		// Save output
		pcl::io::savePLYFile(fileName2, *cloud_out);
	}
	//pcl::io::savePLYFile(fileName2, *cloud);
}