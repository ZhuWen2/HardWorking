//------------------------去噪对比-----------------------------
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/generate.h> // 生成均匀分布点云
#include <pcl/common/random.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/bilateral.h>//双边滤波
#include <pcl/common/gaussian.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/median_filter.h>
//#include<pcl/kdtree/kdtree_flann.h>//双边中的kdtree
//#include<pcl/search/search.h>
//#include<pcl/search/kdtree.h>


using namespace std;

int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\123456\\Desktop\\测试工件\\test\\01-11a.pcd", *cloud);
	
	//统计滤波
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
	//sor.setInputCloud(cloud);                           //设置待滤波的点云
	//sor.setMeanK(50);                               //设置在进行统计时考虑的临近点个数
	//sor.setStddevMulThresh(1.0);                      //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
	//sor.filter(*cloud_filtered);                    //滤波结果存储到cloud_filtered

	//半径滤波
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  //创建滤波器对象
	//pcFilter.setInputCloud(cloud);             //设置待滤波的点云
	//pcFilter.setRadiusSearch(0.8);               // 设置搜索半径
	//pcFilter.setMinNeighborsInRadius(2);      // 设置一个内点最少的邻居数目
	//pcFilter.filter(*cloud_filtered);        //滤波结果存储到cloud_filtered

	//直通滤波(有问题 调参)
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(cloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(0.0, 0.024); //设置Z轴范围为0.0到2.4m
	//pass.filter(*cloud_filtered);

	//体素滤波
	//pcl::VoxelGrid<pcl::PointXYZ> sor;
	//sor.setInputCloud(cloud);
	//sor.setLeafSize(0.01f, 0.01f, 0.01f);//设置体素大小，米为单位
	//sor.filter(*cloud_filtered);

	//均匀采样
	//pcl::UniformSampling<pcl::PointXYZ>us;	//创建滤波器对象
	//us.setInputCloud(cloud);				//设置待滤波点云
	//us.setRadiusSearch(0.05f);				//设置滤波球体半径
	//us.filter(*cloud_filtered);		//执行滤波，保存滤波结果于cloud_filtered

	//双边滤波(没有强度，用不了)
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	//pcl::BilateralFilter<pcl::PointXYZI> bf;
	//bf.setInputCloud(cloud);
	//bf.setSearchMethod(tree);
	//bf.setHalfSize(0.1);	// 设置高斯双边滤波窗口的一半大小,即搜索半径。
	//bf.setStdDev(0.03);		// 设置标准差参数
	//bf.filter(*outcloud);

	//高斯
	/*pcl::PointCloudpcl::PointXYZRGB::Ptr cloud(new pcl::PointCloudpcl::PointXYZRGB);     
	pcl::PointCloudpcl::PointXYZRGB::Ptr cloud_filtered(new pcl::PointCloudpcl::PointXYZRGB);*/
	//pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\123456\\Desktop\\测试工件\\test\\01-11a.pcd", *cloud);
    // 创建高斯滤波器
	//pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
	//(*kernel).setSigma(4);
	//(*kernel).setThresholdRelativeToSigma(4);
	//cout << "ffffff" << endl;
    // 创建高斯滤波对象
    //pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	//convolution.setKernel(*kernel);
    // 执行高斯滤波
	//cout << "ffffff" << endl;
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	//(*kdtree).setInputCloud(cloud);
	//convolution.setInputCloud(cloud);
	//convolution.setSearchMethod(kdtree);
	//convolution.setRadiusSearch(5);
	//cout << "ffffff" << endl;
	//convolution.convolve(*outputcloud);
	//cout << "ffffff" << endl;
 //   // 保存滤波后的点云数据
 //   pcl::io::savePCDFile("C:\\Users\\123456\\Desktop\\测试工件\\test\\01-11b.pcd", *outputcloud);

	//中值(需使用有序点云，用不了)
	//创建中值滤波器对象
	//pcl::MedianFilter<pcl::PointXYZ>median_filter;
	//median_filter.setInputCloud(cloud);
	//median_filter.setWindowSize(1);
	////执行中值滤波
	//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//median_filter.filter(*filtered_cloud);

	pcl::io::savePCDFile("C:\\Users\\123456\\Desktop\\测试工件\\test\\01-11b.pcd", *cloud_filtered);
	return 0;
}

