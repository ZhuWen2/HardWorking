//------------------------ȥ��Ա�-----------------------------
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/generate.h> // ���ɾ��ȷֲ�����
#include <pcl/common/random.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/bilateral.h>//˫���˲�
#include <pcl/common/gaussian.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/median_filter.h>
//#include<pcl/kdtree/kdtree_flann.h>//˫���е�kdtree
//#include<pcl/search/search.h>
//#include<pcl/search/kdtree.h>


using namespace std;

int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\123456\\Desktop\\���Թ���\\test\\01-11a.pcd", *cloud);
	
	//ͳ���˲�
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //�����˲�������
	//sor.setInputCloud(cloud);                           //���ô��˲��ĵ���
	//sor.setMeanK(50);                               //�����ڽ���ͳ��ʱ���ǵ��ٽ������
	//sor.setStddevMulThresh(1.0);                      //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ���������˱�׼�Ҳ���������std_mul
	//sor.filter(*cloud_filtered);                    //�˲�����洢��cloud_filtered

	//�뾶�˲�
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  //�����˲�������
	//pcFilter.setInputCloud(cloud);             //���ô��˲��ĵ���
	//pcFilter.setRadiusSearch(0.8);               // ���������뾶
	//pcFilter.setMinNeighborsInRadius(2);      // ����һ���ڵ����ٵ��ھ���Ŀ
	//pcFilter.filter(*cloud_filtered);        //�˲�����洢��cloud_filtered

	//ֱͨ�˲�(������ ����)
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(cloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(0.0, 0.024); //����Z�᷶ΧΪ0.0��2.4m
	//pass.filter(*cloud_filtered);

	//�����˲�
	//pcl::VoxelGrid<pcl::PointXYZ> sor;
	//sor.setInputCloud(cloud);
	//sor.setLeafSize(0.01f, 0.01f, 0.01f);//�������ش�С����Ϊ��λ
	//sor.filter(*cloud_filtered);

	//���Ȳ���
	//pcl::UniformSampling<pcl::PointXYZ>us;	//�����˲�������
	//us.setInputCloud(cloud);				//���ô��˲�����
	//us.setRadiusSearch(0.05f);				//�����˲�����뾶
	//us.filter(*cloud_filtered);		//ִ���˲��������˲������cloud_filtered

	//˫���˲�(û��ǿ�ȣ��ò���)
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	//pcl::BilateralFilter<pcl::PointXYZI> bf;
	//bf.setInputCloud(cloud);
	//bf.setSearchMethod(tree);
	//bf.setHalfSize(0.1);	// ���ø�˹˫���˲����ڵ�һ���С,�������뾶��
	//bf.setStdDev(0.03);		// ���ñ�׼�����
	//bf.filter(*outcloud);

	//��˹
	/*pcl::PointCloudpcl::PointXYZRGB::Ptr cloud(new pcl::PointCloudpcl::PointXYZRGB);     
	pcl::PointCloudpcl::PointXYZRGB::Ptr cloud_filtered(new pcl::PointCloudpcl::PointXYZRGB);*/
	//pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\123456\\Desktop\\���Թ���\\test\\01-11a.pcd", *cloud);
    // ������˹�˲���
	//pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
	//(*kernel).setSigma(4);
	//(*kernel).setThresholdRelativeToSigma(4);
	//cout << "ffffff" << endl;
    // ������˹�˲�����
    //pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	//convolution.setKernel(*kernel);
    // ִ�и�˹�˲�
	//cout << "ffffff" << endl;
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	//(*kdtree).setInputCloud(cloud);
	//convolution.setInputCloud(cloud);
	//convolution.setSearchMethod(kdtree);
	//convolution.setRadiusSearch(5);
	//cout << "ffffff" << endl;
	//convolution.convolve(*outputcloud);
	//cout << "ffffff" << endl;
 //   // �����˲���ĵ�������
 //   pcl::io::savePCDFile("C:\\Users\\123456\\Desktop\\���Թ���\\test\\01-11b.pcd", *outputcloud);

	//��ֵ(��ʹ��������ƣ��ò���)
	//������ֵ�˲�������
	//pcl::MedianFilter<pcl::PointXYZ>median_filter;
	//median_filter.setInputCloud(cloud);
	//median_filter.setWindowSize(1);
	////ִ����ֵ�˲�
	//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//median_filter.filter(*filtered_cloud);

	pcl::io::savePCDFile("C:\\Users\\123456\\Desktop\\���Թ���\\test\\01-11b.pcd", *cloud_filtered);
	return 0;
}

