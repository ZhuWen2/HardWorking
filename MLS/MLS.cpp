#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree����������ඨ���ͷ�ļ�
#include <pcl/surface/mls.h>        //��С���˷�ƽ�������ඨ��ͷ�ļ�
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
	fileName1 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\multiway_registration1.ply";
	fileName2 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\multiway_registration1_rong.ply";
	pcl::io::loadPLYFile(fileName1, *cloud);
	for (int i = 1; i < 2; ++i) {
		/*fileName1 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����������" + std::to_string(i) + ".ply";
		fileName2 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\a����������" + std::to_string(i) + ".ply";*/
		
		pcl::io::loadPLYFile(fileName1, *cloud);
		

		std::cout << cloud->size() << std::endl;
		
		// ������С����ʵ�ֵĶ���mls
		

		mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���

		// Set parameters
		mls.setInputCloud(cloud);
		
		mls.setPolynomialOrder(5); //����false�����Լ���ƽ��
		mls.setSearchMethod(tree);
		mls.setSearchRadius(4); //��λm������������ϵ�K���ڰ뾶
		//mls.setNumberOfThreads(4);
		// Reconstruct
		//cloud->clear();
		mls.process(*cloud_out); //���
		//std::cout << "sss" << std::endl;





		//// ���� KD-Tree
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		//// Output has the PointNormal type in order to store the normals calculated by MLS
		//pcl::PointCloud<pcl::PointNormal> mls_points;

		//// ������С����ʵ�ֵĶ���mls
		//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

		//mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���

		//// Set parameters
		//mls.setInputCloud(cloud);
		//mls.setPolynomialOrder(3); //����false�����Լ���ƽ��
		//mls.setSearchMethod(tree);
		//mls.setSearchRadius(5); //
		//mls.setNumberOfThreads(4);
		//// Reconstruct
		//mls.process(mls_points); //���

		// Save output
		pcl::io::savePLYFile(fileName2, *cloud_out);
	}
	//pcl::io::savePLYFile(fileName2, *cloud);
}