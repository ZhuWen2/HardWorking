#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>//体素下采样滤波
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h> //fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_svd.h> //奇异矩阵分解计算变换矩阵

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
    //-------------------------法向量估计-----------------------
    pointnormal::Ptr normals(new pointnormal);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(input_cloud);
    //n.setNumberOfThreads(2);//设置openMP的线程数
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);
    cout << "hh" << endl;
    //------------------FPFH估计-------------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    //f.setNumberOfThreads(2); //指定8核计算
    f.setInputCloud(input_cloud);
    f.setInputNormals(normals);
    f.setSearchMethod(tree);
    f.setKSearch(10);
    f.compute(*fpfh);
    cout << "hh" << endl;
    return fpfh;

}

int
main(int argc, char** argv)
{
    clock_t start, end, time;
    start = clock();
    pointcloud::Ptr source_cloud(new pointcloud);
    pointcloud::Ptr target_cloud(new pointcloud);
    pcl::io::loadPLYFile<pcl::PointXYZ>("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\PT2.ply", *source_cloud);
    pcl::io::loadPLYFile<pcl::PointXYZ>("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\PT22.ply", *target_cloud);
    ////---------------------------去除源点云的NAN点------------------------
    //vector<int> indices_src; //保存去除的点的索引
    //pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
    //cout << "remove *source_cloud nan" << endl;
    //-------------------------源点云下采样滤波-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vs;
    vs.setLeafSize(2, 2, 2);
    vs.setInputCloud(source_cloud);
    pointcloud::Ptr source(new pointcloud);
    vs.filter(*source);
    cout << "down size *source_cloud from " << source_cloud->size() << " to " << source->size() << endl;

    //--------------------------去除目标点云的NAN点--------------------
    vector<int> indices_tgt; //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
    cout << "remove *target_cloud nan" << endl;
    //----------------------目标点云下采样滤波-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vt;
    vt.setLeafSize(2, 2, 2);
    vt.setInputCloud(target_cloud);
    pointcloud::Ptr target(new pointcloud);
    vt.filter(*target);
    cout << "down size *target_cloud from " << target_cloud->size() << " to " << target->size() << endl;
    //---------------计算源点云和目标点云的FPFH------------------------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    //fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);


    pointnormal::Ptr normals1(new pointnormal);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(source);
    //n.setNumberOfThreads(2);//设置openMP的线程数
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals1);
    cout << "hh" << endl;
    //------------------FPFH估计-------------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    //f.setNumberOfThreads(2); //指定8核计算
    f.setInputCloud(source);
    f.setInputNormals(normals1);
    f.setSearchMethod(tree);
    f.setKSearch(10);
    f.compute(*fpfh);
    cout << "hh" << endl;
    fpfhFeature::Ptr source_fpfh = fpfh;


    
    //fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);

    pointnormal::Ptr normals2(new pointnormal);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n1;
    n1.setInputCloud(target);
    //n.setNumberOfThreads(2);//设置openMP的线程数
    n1.setSearchMethod(tree);
    n1.setKSearch(10);
    n1.compute(*normals2);
    cout << "hh" << endl;
    //------------------FPFH估计-------------------------------
    fpfhFeature::Ptr fpfh1(new fpfhFeature);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f1;
    //f.setNumberOfThreads(2); //指定8核计算
    f1.setInputCloud(target);
    f1.setInputNormals(normals2);
    f1.setSearchMethod(tree);
    f1.setKSearch(10);
    f1.compute(*fpfh1);
    fpfhFeature::Ptr target_fpfh = fpfh1;
    
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    crude_cor_est.setInputSource(source_fpfh);
    crude_cor_est.setInputTarget(target_fpfh);
    //crude_cor_est.determineCorrespondences(*cru_correspondences,50);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences,1);
    cout << "crude size is:" << cru_correspondences->size() << endl;
    /*
    *   //  该方法与*******111111111*********相同
        boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);

        float a, threshold = 1;
        int total = 0;
        for (int i = 0; i < cru_correspondences->size(); i++)
        {
            a = cru_correspondences->at(i).distance;
            if (a <= treshold*threshold)
            {
                correspondences->push_back(cru_correspondences->at(i));
                total++;
            }

        }
        cout << "After filter total correspondences is" << total << endl;

        */
        //-----------------------------奇异矩阵分解求变换矩阵-----------
    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr trans(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>);

    trans->estimateRigidTransformation(*source, *target, *cru_correspondences, Transform);

    cout << "SVD求得的变换矩阵为：\n" << Transform << endl;

    end = clock();
    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;
    pcl::transformPointCloud(*source, *source, Transform);
    pcl::io::savePCDFile("left_sub_reg_right.pcd", *source, false);


    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
    viewer->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
    // 对目标点云着色可视化 (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    // 对源点云着色可视化 (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(source, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(source, input_color, "input cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
    //对应关系可视化
    viewer->addCorrespondences<pcl::PointXYZ>(source, target, *cru_correspondences, "correspondence");
    //viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }


    return 0;
}

