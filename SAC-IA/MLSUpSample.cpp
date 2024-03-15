#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>//体素下采样滤波
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/features/fpfh_omp.h> //fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/ia_ransac.h>//sac_ia算法
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h> 
#include <boost/shared_ptr.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件
#include<pcl/search/impl/kdtree.hpp>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
    //-------------------------法向量估计-----------------------
    pointnormal::Ptr normals(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(input_cloud);
    n.setNumberOfThreads(8);//设置openMP的线程数
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);
    //------------------FPFH估计-------------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    f.setNumberOfThreads(8); //指定8核计算
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
    //--------创建两个显示窗口并设置背景颜色------------
    int v1, v2;
    viewer.createViewPort(0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.setBackgroundColor(0.05, 0, 0, v2);
    //-----------给点云添加颜色-------------------------
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 255, 0);
    //----------添加点云到显示窗口----------------------
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
    // 创建 KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());

    for (int i = 2; i < 3; ++i) {

        clock_t start, end, time;
        start = clock();


        string fileName1, fileName2, fileName3;
        fileName3 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\a" + to_string(i) + ".ply";
        if (i == 2) {
            fileName1 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\China" + to_string(i) + ".ply";
            fileName2 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\China" + to_string(i + 1) + ".ply";
        }
        else {
            fileName1 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\a" + to_string(i - 1) + ".ply";
            fileName2 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\China" + to_string(i + 1) + ".ply";
        }



        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName1, *source_cloud);
        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName2, *target_cloud);
        //---------------------------去除源点云的NAN点------------------------
        vector<int> indices_src; //保存去除的点的索引
        pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
        cout << "remove *source_cloud nan" << endl;
        //-------------------------源点云下采样滤波-------------------------
        pcl::VoxelGrid<pcl::PointXYZ> vs;
        vs.setLeafSize(2, 2, 2);
        vs.setInputCloud(source_cloud);

        vs.filter(*source);
        cout << "down size *source_cloud from " << source_cloud->size() << " to " << source->size() << endl;

        ////--------------------------去除目标点云的NAN点--------------------
        vector<int> indices_tgt; //保存去除的点的索引
        pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
        cout << "remove *target_cloud nan" << endl;
        //----------------------目标点云下采样滤波-------------------------
        pcl::VoxelGrid<pcl::PointXYZ> vt;
        vt.setLeafSize(2, 2, 2);
        vt.setInputCloud(target_cloud);

        vt.filter(*target);
        //pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\降采样.ply", *target);
        cout << "down size *target_cloud from " << target_cloud->size() << " to " << target->size() << endl;








       
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<pcl::PointNormal> mls_points;

        // 定义最小二乘实现的对象mls
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

        mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

        // Set parameters
        mls.setInputCloud(target);
        mls.setPolynomialOrder(2); //设置false，可以加速平滑
        mls.setSearchMethod(tree1);
        mls.setSearchRadius(3); //单位mm，设置用于拟合的K近邻半径
        mls.setNumberOfThreads(4);
    
        //Upsampling 采样的方法还有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE); //对点云进行上采样
        mls.setUpsamplingRadius(1);    // r2
        mls.setUpsamplingStepSize(0.8);  // r3
        mls.process(mls_points); //输出
        pcl::io::savePLYFile("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\上采样.ply", mls_points);





        // Set parameters
        mls.setInputCloud(source);
        mls.setPolynomialOrder(2); //设置false，可以加速平滑
        mls.setSearchMethod(tree1);
        mls.setSearchRadius(3); //单位mm，设置用于拟合的K近邻半径
        mls.setNumberOfThreads(4);

        //Upsampling 采样的方法还有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE); //对点云进行上采样
        mls.setUpsamplingRadius(1);    // r2
        mls.setUpsamplingStepSize(0.8);  // r3
        mls.process(mls_points); //输出
        pcl::io::savePLYFile("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\上采样1.ply", mls_points);

    }

    return 0;
}
