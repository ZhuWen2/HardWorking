#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>//�����²����˲�
#include <pcl/features/normal_3d_omp.h>//ʹ��OMP��Ҫ��ӵ�ͷ�ļ�
#include <pcl/features/fpfh_omp.h> //fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/ia_ransac.h>//sac_ia�㷨
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h> 
#include <boost/shared_ptr.hpp>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/normal_space.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;



void visualize_pcd(pointcloud::Ptr pcd_src, pointcloud::Ptr pcd_tgt, pointcloud::Ptr pcd_final)
{

    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    //--------����������ʾ���ڲ����ñ�����ɫ------------
    int v1, v2;
    viewer.createViewPort(0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.setBackgroundColor(0.05, 0, 0, v2);
    //-----------�����������ɫ-------------------------
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 255, 0);
    //----------��ӵ��Ƶ���ʾ����----------------------
    viewer.addPointCloud(pcd_src, src_h, "source cloud", v1);
    viewer.addPointCloud(pcd_tgt, tgt_h, "target cloud", v1);
    viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud", v2);
    viewer.addPointCloud(pcd_final, final_h, "final cloud", v2);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char** argv)
{
    pointcloud::Ptr source_cloud(new pointcloud);
    pointcloud::Ptr target_cloud(new pointcloud);
    pointcloud::Ptr source(new pointcloud);
    pointcloud::Ptr target(new pointcloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pointnormal::Ptr normals(new pointnormal);
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pointnormal::Ptr normals1(new pointnormal);
    fpfhFeature::Ptr fpfh1(new fpfhFeature);
    pointcloud::Ptr align(new pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n1;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f1;


        clock_t start, end, time;
        start = clock();


        string fileName1, fileName2, fileName3;
        fileName3 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\����������2.ply";
       
        fileName1 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\����2.ply";
        
      
       



        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName1, *source_cloud);
       
        //---------------------------ȥ��Դ���Ƶ�NAN��------------------------
        vector<int> indices_src; //����ȥ���ĵ������
        pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
        cout << "remove *source_cloud nan" << endl;
        //-------------------------Դ�����²����˲�-------------------------
        //pcl::VoxelGrid<pcl::PointXYZ> vs;
        /*vs.setLeafSize(2, 2, 2);
        vs.setInputCloud(source_cloud);

        vs.filter(*source);
        cout << "down size *source_cloud from " << source_cloud->size() << " to " << source->size() << endl;*/




        //pcl::RandomSample<pcl::PointXYZ> vs;    //�����˲�������
        //vs.setInputCloud(source_cloud);                //���ô��˲�����
        //vs.setSample(15000);                    //�����²������Ƶĵ���
        ////vs.setSeed(1);                        //��������������ӵ�
        //vs.filter(*source);                    //ִ���²����˲��������˲������cloud_sub
        //pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\������.ply", *source);
        




        //// �������Ȳ�����ģ�壩����󣬵���������Ϊ pcl::PointXYZRGB
        //pcl::UniformSampling<pcl::PointXYZ> us;
        //// ����������ƣ�ע�⣺�˴�������ǵ�������������ָ��
        //us.setInputCloud(source_cloud);
        //// ���ò����뾶����ֵԽ��Խϡ��
        //us.setRadiusSearch(0.6);
        //// ִ�й��ˣ������������ĵ������ݣ�ע�⣺�˴�������ǵ��������
        //us.filter(*source);
        //pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\������.ply", *source);



        // ������������ķ�����������
 // // ����omp���м��٣������ÿ���OpenMP
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne; 
        ne.setNumberOfThreads(8);
        //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        // ����һ���յ�kdtree���󣬲��������ݸ����߹��ƶ���
        // ���ڴ�����������������ݵ���������kdtree
        /*pcl::search::KdTree<pcl::PointXYZ>::Ptr \
            tree(new pcl::search::KdTree<pcl::PointXYZ>());*/
        // ��������Ʒ��ߵĵ������ݣ�����ָ��
        ne.setInputCloud(source_cloud);
        // ����kdtree��������ָ��
        ne.setSearchMethod(tree);
        // �������������뾶
        //ne.setRadiusSearch(1);    // ���ð뾶ʱ��Ҫ���ǵ����ƿռ���
        // // Ҳ������������ڵ����
         ne.setKSearch(16);
        // �����ӵ�Դ�㣬���ڵ������Ʒ���ָ���ӵ㣩��Ĭ�ϣ�0��0��0��
        ne.setViewPoint(0, 0, 0);
        // ���㷨������
        ne.compute(*normals);


        // ͨ��concatenateFields������point��normal��������γ�PointNormal��������
        pcl::PointCloud<pcl::PointNormal>::Ptr \
            cloud_with_normal(new pcl::PointCloud<pcl::PointNormal>());
        pcl::PointCloud<pcl::PointNormal>::Ptr \
            cloud_with_normal_sampled(new pcl::PointCloud<pcl::PointNormal>());
        pcl::concatenateFields(*source_cloud, *normals, *cloud_with_normal);


        // ��������ռ������ģ�壩�����
        pcl::NormalSpaceSampling<pcl::PointNormal, pcl::Normal> nss;
        // ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���
        const int kBinNum = 16;
        nss.setBins(kBinNum, kBinNum, kBinNum);
        // ����������������ƣ��˴����Գ�������Ϊtrue
        nss.setKeepOrganized(false);
        // ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
        nss.setSeed(0);   // random seed
        // ����������ĵ�������
        nss.setInputCloud(cloud_with_normal);
        // �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
        nss.setNormals(normals);
        // ���ò�����������Ŀ����Ƶ���������
        const float kSampleRatio = 0.1f;
        nss.setSample(cloud_with_normal->size() * kSampleRatio);
        // ִ�в����������������
        nss.filter(*cloud_with_normal_sampled);
        pcl::io::savePLYFileASCII(fileName3, *cloud_with_normal_sampled);

    return 0;
}
