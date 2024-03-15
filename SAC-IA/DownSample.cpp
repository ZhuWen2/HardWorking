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


        clock_t start, end, time;
        start = clock();


        string fileName1, fileName2, fileName3;
        fileName3 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\工件1配准\\降采样工件2.ply";
       
        fileName1 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\工件1配准\\工件2.ply";
        
      
       



        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName1, *source_cloud);
       
        //---------------------------去除源点云的NAN点------------------------
        vector<int> indices_src; //保存去除的点的索引
        pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
        cout << "remove *source_cloud nan" << endl;
        //-------------------------源点云下采样滤波-------------------------
        //pcl::VoxelGrid<pcl::PointXYZ> vs;
        /*vs.setLeafSize(2, 2, 2);
        vs.setInputCloud(source_cloud);

        vs.filter(*source);
        cout << "down size *source_cloud from " << source_cloud->size() << " to " << source->size() << endl;*/




        //pcl::RandomSample<pcl::PointXYZ> vs;    //创建滤波器对象
        //vs.setInputCloud(source_cloud);                //设置待滤波点云
        //vs.setSample(15000);                    //设置下采样点云的点数
        ////vs.setSeed(1);                        //设置随机函数种子点
        //vs.filter(*source);                    //执行下采样滤波，保存滤波结果于cloud_sub
        //pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\降采样.ply", *source);
        




        //// 创建均匀采样（模板）类对象，点数据类型为 pcl::PointXYZRGB
        //pcl::UniformSampling<pcl::PointXYZ> us;
        //// 设置输入点云，注意：此处传入的是点云类对象的智能指针
        //us.setInputCloud(source_cloud);
        //// 设置采样半径，数值越大越稀疏
        //us.setRadiusSearch(0.6);
        //// 执行过滤，并带出处理后的点云数据，注意：此处传入的是点云类对象
        //us.filter(*source);
        //pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\降采样.ply", *source);



        // 创建基于邻域的法向估计类对象
 // // 基于omp并行加速，需配置开启OpenMP
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne; 
        ne.setNumberOfThreads(8);
        //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        // 创建一个空的kdtree对象，并把它传递给法线估计对象，
        // 用于创建基于输入点云数据的邻域搜索kdtree
        /*pcl::search::KdTree<pcl::PointXYZ>::Ptr \
            tree(new pcl::search::KdTree<pcl::PointXYZ>());*/
        // 传入待估计法线的点云数据，智能指针
        ne.setInputCloud(source_cloud);
        // 传入kdtree对象，智能指针
        ne.setSearchMethod(tree);
        // 设置邻域搜索半径
        //ne.setRadiusSearch(1);    // 设置半径时，要考虑到点云空间间距
        // // 也可以设置最近邻点个数
         ne.setKSearch(16);
        // 设置视点源点，用于调整点云法向（指向视点），默认（0，0，0）
        ne.setViewPoint(0, 0, 0);
        // 计算法线数据
        ne.compute(*normals);


        // 通过concatenateFields函数将point和normal组合起来形成PointNormal点云数据
        pcl::PointCloud<pcl::PointNormal>::Ptr \
            cloud_with_normal(new pcl::PointCloud<pcl::PointNormal>());
        pcl::PointCloud<pcl::PointNormal>::Ptr \
            cloud_with_normal_sampled(new pcl::PointCloud<pcl::PointNormal>());
        pcl::concatenateFields(*source_cloud, *normals, *cloud_with_normal);


        // 创建法向空间采样（模板）类对象
        pcl::NormalSpaceSampling<pcl::PointNormal, pcl::Normal> nss;
        // 设置xyz三个法向空间的分类组数，此处设置为一致，根据具体场景可以调整
        const int kBinNum = 16;
        nss.setBins(kBinNum, kBinNum, kBinNum);
        // 如果传入的是有序点云，此处可以尝试设置为true
        nss.setKeepOrganized(false);
        // 设置随机种子，这样可以保证同样的输入可以得到同样的结果，便于debug分析
        nss.setSeed(0);   // random seed
        // 传入待采样的点云数据
        nss.setInputCloud(cloud_with_normal);
        // 传入用于采样分析的法线数据，需与传入点云数据一一对应
        nss.setNormals(normals);
        // 设置采样总数，即目标点云的总数据量
        const float kSampleRatio = 0.1f;
        nss.setSample(cloud_with_normal->size() * kSampleRatio);
        // 执行采样并带出采样结果
        nss.filter(*cloud_with_normal_sampled);
        pcl::io::savePLYFileASCII(fileName3, *cloud_with_normal_sampled);

    return 0;
}
