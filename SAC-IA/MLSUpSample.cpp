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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>        //��С���˷�ƽ�������ඨ��ͷ�ļ�
#include<pcl/search/impl/kdtree.hpp>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
    //-------------------------����������-----------------------
    pointnormal::Ptr normals(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(input_cloud);
    n.setNumberOfThreads(8);//����openMP���߳���
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);
    //------------------FPFH����-------------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    f.setNumberOfThreads(8); //ָ��8�˼���
    f.setInputCloud(input_cloud);
    f.setInputNormals(normals);
    f.setSearchMethod(tree);
    f.setKSearch(10);
    f.compute(*fpfh);

    return fpfh;

}

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
    // ���� KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());

    for (int i = 2; i < 3; ++i) {

        clock_t start, end, time;
        start = clock();


        string fileName1, fileName2, fileName3;
        fileName3 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\a" + to_string(i) + ".ply";
        if (i == 2) {
            fileName1 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\China" + to_string(i) + ".ply";
            fileName2 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\China" + to_string(i + 1) + ".ply";
        }
        else {
            fileName1 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\a" + to_string(i - 1) + ".ply";
            fileName2 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\China" + to_string(i + 1) + ".ply";
        }



        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName1, *source_cloud);
        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName2, *target_cloud);
        //---------------------------ȥ��Դ���Ƶ�NAN��------------------------
        vector<int> indices_src; //����ȥ���ĵ������
        pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
        cout << "remove *source_cloud nan" << endl;
        //-------------------------Դ�����²����˲�-------------------------
        pcl::VoxelGrid<pcl::PointXYZ> vs;
        vs.setLeafSize(2, 2, 2);
        vs.setInputCloud(source_cloud);

        vs.filter(*source);
        cout << "down size *source_cloud from " << source_cloud->size() << " to " << source->size() << endl;

        ////--------------------------ȥ��Ŀ����Ƶ�NAN��--------------------
        vector<int> indices_tgt; //����ȥ���ĵ������
        pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
        cout << "remove *target_cloud nan" << endl;
        //----------------------Ŀ������²����˲�-------------------------
        pcl::VoxelGrid<pcl::PointXYZ> vt;
        vt.setLeafSize(2, 2, 2);
        vt.setInputCloud(target_cloud);

        vt.filter(*target);
        //pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\������.ply", *target);
        cout << "down size *target_cloud from " << target_cloud->size() << " to " << target->size() << endl;








       
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<pcl::PointNormal> mls_points;

        // ������С����ʵ�ֵĶ���mls
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

        mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���

        // Set parameters
        mls.setInputCloud(target);
        mls.setPolynomialOrder(2); //����false�����Լ���ƽ��
        mls.setSearchMethod(tree1);
        mls.setSearchRadius(3); //��λmm������������ϵ�K���ڰ뾶
        mls.setNumberOfThreads(4);
    
        //Upsampling �����ķ������� DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE); //�Ե��ƽ����ϲ���
        mls.setUpsamplingRadius(1);    // r2
        mls.setUpsamplingStepSize(0.8);  // r3
        mls.process(mls_points); //���
        pcl::io::savePLYFile("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\�ϲ���.ply", mls_points);





        // Set parameters
        mls.setInputCloud(source);
        mls.setPolynomialOrder(2); //����false�����Լ���ƽ��
        mls.setSearchMethod(tree1);
        mls.setSearchRadius(3); //��λmm������������ϵ�K���ڰ뾶
        mls.setNumberOfThreads(4);

        //Upsampling �����ķ������� DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE); //�Ե��ƽ����ϲ���
        mls.setUpsamplingRadius(1);    // r2
        mls.setUpsamplingStepSize(0.8);  // r3
        mls.process(mls_points); //���
        pcl::io::savePLYFile("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\�ϲ���1.ply", mls_points);

    }

    return 0;
}
