//------------------------1 ��Ӿ�������-----------------------------
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/generate.h> // ���ɾ��ȷֲ�����
#include <pcl/common/random.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\123456\\Desktop\\���Թ���\\test\\01-11.pcd", *cloud);
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	// ���þ��ȷֲ���ز���
	float xmin = minPt.x;
	float xmax = maxPt.x;
	float ymin = minPt.y;
	float ymax = maxPt.y;
	float zmin = minPt.z;
	float zmax = maxPt.z;

	int size = (*cloud).size() * 0.1; // ���ɾ��ȷֲ�������ĸ���������������á�

	// ---------------------------����������ȷֲ��ĵ�������------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::UniformGenerator<float> > generator;
	uint32_t seed = static_cast<uint32_t> (time(NULL));
	pcl::common::UniformGenerator<float>::Parameters x_params(xmin, xmax, seed++); // ����Xγ�Ⱦ��ȷֲ���ֵ�����ֵ����Сֵ
	generator.setParametersForX(x_params);
	pcl::common::UniformGenerator<float>::Parameters y_params(ymin, ymax, seed++);// ����Xγ�Ⱦ��ȷֲ���ֵ�����ֵ����Сֵ
	generator.setParametersForY(y_params);
	pcl::common::UniformGenerator<float>::Parameters z_params(zmin, zmax, seed++);// ����Xγ�Ⱦ��ȷֲ���ֵ�����ֵ����Сֵ
	generator.setParametersForZ(z_params);
	generator.fill(size, 1, *uniform_cloud); // ���������sizeΪ���Ƶ�with��1Ϊ���Ƶ�height
	// -----------------------------------�ϲ�����---------------------------------------------
	*uniform_cloud += *cloud;
	// -----------------------------------�������---------------------------------------------
	pcl::io::savePCDFile("C:\\Users\\123456\\Desktop\\���Թ���\\test\\01-11a.pcd", *uniform_cloud);
	// ----------------------------------������ӻ�--------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));
	viewer->setWindowName("������Ӿ��ȷֲ����������");
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("uniformed point clouds", 10, 10, "v2_text", v2);

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(uniform_cloud, "uniform", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "uniform", v2);
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}

