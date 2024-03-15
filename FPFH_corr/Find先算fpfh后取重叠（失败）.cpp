#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp算法
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>//体素下采样滤波
#include <pcl/features/fpfh_omp.h> //fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_svd.h> //奇异矩阵分解计算变换矩阵
#include <pcl/registration/ia_ransac.h>//sac_ia算法
#include <fstream>
#include <time.h>
#include <unordered_set>
clock_t whole_time = 0;


using namespace std;

unordered_set<double> sourceset;
unordered_set<double> targetset;

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


void delete_wrong_corr(boost::shared_ptr<pcl::Correspondences> cru_correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target) {

    cout << "删除前crude size is:" << cru_correspondences->size() << endl;
    int slow = 0;
    for (int fast = 0; fast < (*cru_correspondences).size(); ++fast) {
        if (
            /*(source->points[(*cru_correspondences)[fast].index_query].x - target->points[(*cru_correspondences)[fast].index_match].x < 1)
            && (source->points[(*cru_correspondences)[fast].index_query].y - target->points[(*cru_correspondences)[fast].index_match].y < 1)
            && (source->points[(*cru_correspondences)[fast].index_query].z - target->points[(*cru_correspondences)[fast].index_match].z < 1)*/
            sqrt(pow((source->points[(*cru_correspondences)[fast].index_query].x - target->points[(*cru_correspondences)[fast].index_match].x), 2)
                + pow((source->points[(*cru_correspondences)[fast].index_query].y - target->points[(*cru_correspondences)[fast].index_match].y), 2)
                + pow((source->points[(*cru_correspondences)[fast].index_query].z - target->points[(*cru_correspondences)[fast].index_match].z), 2)) < 3
            ) {
            (*cru_correspondences)[slow].index_query = (*cru_correspondences)[fast].index_query;
            (*cru_correspondences)[slow].index_match = (*cru_correspondences)[fast].index_match;
            ++slow;
        }
    }
    (*cru_correspondences).resize(slow - 1);

    cout << "删除后crude size is:" << cru_correspondences->size() << endl;
}


void save_corr(boost::shared_ptr<pcl::Correspondences> cru_correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, int a, int b) {
    string filename1 = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\overlap_cloud\\PT" + to_string(a) + "_" + to_string(b) + "_" + to_string(a) + ".txt";
    string filename2 = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\overlap_cloud\\PT" + to_string(a) + "_" + to_string(b) + "_" + to_string(b) + ".txt";
    std::ofstream source_cloud;
    source_cloud.open(filename1, std::ios::out);
    std::ofstream target_cloud;
    target_cloud.open(filename2, std::ios::out);
    for (int i = 0; i < (*cru_correspondences).size(); ++i) {
        source_cloud << source->points[(*cru_correspondences)[i].index_query].x << " "
            << source->points[(*cru_correspondences)[i].index_query].y << " "
            << source->points[(*cru_correspondences)[i].index_query].z << " "
            << std::endl;


        target_cloud << target->points[(*cru_correspondences)[i].index_match].x << " "
            << target->points[(*cru_correspondences)[i].index_match].y << " "
            << target->points[(*cru_correspondences)[i].index_match].z << " "
            << std::endl;
    }
    source_cloud.close();
    target_cloud.close();
}


void find_fpfh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, int a, int b) {
    clock_t start, end, time;
    start = clock();

    ////---------------------------去除源点云的NAN点------------------------
    //vector<int> indices_src; //保存去除的点的索引
    //pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
    //cout << "remove *source_cloud nan" << endl;
    //-------------------------源点云下采样滤波-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vs;
    vs.setLeafSize(0.5, 0.5, 0.5);
    vs.setInputCloud(cloud1);
    pointcloud::Ptr source(new pointcloud);
    vs.filter(*source);
    cout << "down size *cloud1 from " << cloud1->size() << " to " << source->size() << endl;

    //--------------------------去除目标点云的NAN点--------------------
    vector<int> indices_tgt; //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices_tgt);
    cout << "remove *target_cloud nan" << endl;
    //----------------------目标点云下采样滤波-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vt;
    vt.setLeafSize(0.5, 0.5, 0.5);
    vt.setInputCloud(cloud2);
    pointcloud::Ptr target(new pointcloud);
    vt.filter(*target);
    cout << "down size *target_cloud from " << cloud2->size() << " to " << target->size() << endl;
    //---------------计算源点云和目标点云的FPFH------------------------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    //fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);


    pointnormal::Ptr normals1(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(source);
    n.setNumberOfThreads(8);//设置openMP的线程数
    n.setSearchMethod(tree);
    n.setKSearch(14);
    n.compute(*normals1);

    //------------------FPFH估计-------------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    f.setNumberOfThreads(8); //指定8核计算
    f.setInputCloud(source);
    f.setInputNormals(normals1);
    f.setSearchMethod(tree);
    f.setKSearch(40);
    f.compute(*fpfh);

    int slow = 0;
    fpfhFeature::Ptr source_fpfh = fpfh;
    
    /*for (int fast = 0; fast < source_fpfh->size(); ++fast) {
        if (sourceset.find(source->points[fast].x) != sourceset.end()) {
            source_fpfh->points[slow++] = source_fpfh->points[fast];
        }
    }
    source_fpfh->resize(slow - 1);
    cout << source_fpfh->size() << endl;
    slow = 0;*/
    //fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);

    pointnormal::Ptr normals2(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n1;
    n1.setInputCloud(target);
    n1.setNumberOfThreads(8);//设置openMP的线程数
    n1.setSearchMethod(tree);
    n1.setKSearch(14);
    n1.compute(*normals2);

    //------------------FPFH估计-------------------------------
    fpfhFeature::Ptr fpfh1(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f1;
    f1.setNumberOfThreads(8); //指定8核计算
    f1.setInputCloud(target);
    f1.setInputNormals(normals2);
    f1.setSearchMethod(tree);
    f1.setKSearch(40);
    f1.compute(*fpfh1);
    fpfhFeature::Ptr target_fpfh = fpfh1;
    /*for (int fast = 0; fast < target_fpfh->size(); ++fast) {
        if (targetset.find(target->points[fast].x) != targetset.end()) {
            target_fpfh->points[slow++] = target_fpfh->points[fast];
        }
    }
    target_fpfh->resize(slow - 1);
    cout << target_fpfh->size() << endl;
    slow = 0;*/


    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    crude_cor_est.setInputSource(source_fpfh);
    crude_cor_est.setInputTarget(target_fpfh);
    //crude_cor_est.determineCorrespondences(*cru_correspondences,50);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences, 2);
   /* for (int fast = 0; fast < (*cru_correspondences).size(); ++fast) {

        cout << source->points[(*cru_correspondences)[fast].index_query].x << "  " << target->points[(*cru_correspondences)[fast].index_match].x << endl;
        
    }*/
    
    clock_t s4 = clock();
    //--------------排除配对距离过大的错误匹配点----------------------------
    delete_wrong_corr(cru_correspondences, source, target);
    clock_t e4 = clock();
    whole_time += e4 - s4;
    //打印对应点距离
    for (int i = 0; i < (*cru_correspondences).size(); ++i) {
        cout << sqrt(pow((source->points[(*cru_correspondences)[i].index_query].x - target->points[(*cru_correspondences)[i].index_match].x), 2) + pow((source->points[(*cru_correspondences)[i].index_query].y - target->points[(*cru_correspondences)[i].index_match].y), 2) + pow((source->points[(*cru_correspondences)[i].index_query].z - target->points[(*cru_correspondences)[i].index_match].z), 2)) << " ";
    }


    clock_t s5 = clock();
    //--------------储存找到的对应点----------------------------
    save_corr(cru_correspondences, source, target, a, b);
    clock_t e5 = clock();
    whole_time += e5 - s5;
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

        //应该用不着sac-ia
        //--------------采样一致性SAC_IA初始配准----------------------------
        //pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
        //sac_ia.setInputSource(source);
        //sac_ia.setSourceFeatures(source_fpfh);
        //sac_ia.setInputTarget(target);
        //sac_ia.setTargetFeatures(target_fpfh);
        //sac_ia.setMinSampleDistance(6);//设置样本之间的最小距离
        //sac_ia.setNumberOfSamples(30);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
        ////sac_ia.setCorrespondenceRandomness(3); //在选择随机特征对应时，设置要使用的邻居的数量;
        ////也就是计算协方差时选择的近邻点个数，该值越大，协防差越精确，但是计算效率越低.(可省)
        ////sac_ia.setErrorFunction();//这个调用是可选的
        //pointcloud::Ptr align(new pointcloud);
        //sac_ia.align(*align);
        //end = clock();
        //pcl::transformPointCloud(*source, *align, sac_ia.getFinalTransformation());
        ////pcl::io::savePLYFile("crou_output.ply", *align);
        //cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << "s" << endl;
        //cout << "\nSAC_IA has converged, score is " << sac_ia.getFitnessScore() << endl;
        //cout << "变换矩阵：\n" << sac_ia.getFinalTransformation() << endl;


        ////-------------------保存sac—ia后的FPFH对应点------------------------------------
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sac1(new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sac2(new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::PointXYZ point;

        //for (int i = 0; i < (*cru_correspondences).size(); ++i) {
        //    point.x = align->points[(*cru_correspondences)[i].index_query].x;
        //    point.y = align->points[(*cru_correspondences)[i].index_query].y;
        //    point.z = align->points[(*cru_correspondences)[i].index_query].z;
        //    cloud_sac1->push_back(point);


        //    point.x = target->points[(*cru_correspondences)[i].index_match].x;
        //    point.y = target->points[(*cru_correspondences)[i].index_match].y;
        //    point.z = target->points[(*cru_correspondences)[i].index_match].z;
        //    cloud_sac2->push_back(point);
        //}
        //pcl::io::savePLYFile("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\sac_ia后的overlap\\PT" + to_string(a) + "_" + to_string(b) + "_" + to_string(a) + ".ply", *align);
        //pcl::io::savePLYFile("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\sac_ia后的overlap\\PT" + to_string(a) + "_" + to_string(b) + "_" + to_string(b) + ".ply", *target);


        //-------------------可视化------------------------------------
        //visualize_pcd(source, target, align);
            //-----------------------------奇异矩阵分解求变换矩阵-----------
    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr trans(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>);

    trans->estimateRigidTransformation(*source, *target, *cru_correspondences, Transform);

    cout << "SVD求得的变换矩阵为：\n" << Transform << endl;

    end = clock();
    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;
    pcl::transformPointCloud(*source, *source, Transform);
    //pcl::io::savePCDFile("left_sub_reg_right.pcd", *source, false);


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



}

double calaPointCloudCoincide(int m, int n, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1)
{
    //计算重合度，小于一定重合度的就不考虑
    clock_t s3 = clock();
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
    core.setInputSource(cloud_src);
    core.setInputTarget(cloud_target);

    boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //共享所有权的智能指针，以kdtree做索引

    core.determineReciprocalCorrespondences(*cor, para1);   //点之间的最大距离,cor对应索引
    double chonghe = double(cor->size()) / double(cloud_src->size());
    
    //把重叠部分的源点云的坐标放进集合，以便之后筛选重叠部分的fpfh
    for (int i = 0; i < (*cor).size(); ++i) {
        sourceset.insert(cloud_src->points[(*cor)[i].index_query].x);
        targetset.insert(cloud_target->points[(*cor)[i].index_match].x);
    }

    
    if (chonghe < 0.1) {
        return 0;
    }

    clock_t e3 = clock();
    whole_time += e3 - s3;

    find_fpfh(cloud_src, cloud_target, m, n);
    cout << chonghe << endl;
    return chonghe;
}


void find_corr() {

    for (int i = 0; i < 9; ++i) {
        clock_t s1 = clock();
        // --------------------加载源点云-----------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
        string fileName1 = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\PT" + to_string(i) + ".ply";
        pcl::io::loadPLYFile(fileName1, *source);
        cout << "从源点云中读取 " << source->size() << " 个点" << endl;
        clock_t e1 = clock();
        whole_time += e1 - s1;
        for (int j = i + 1; j < 10; ++j) {

            clock_t s2 = clock();
            // -------------------加载目标点云----------------------
            pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
            string fileName2 = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\PT" + to_string(j) + ".ply";
            pcl::io::loadPLYFile(fileName2, *target);
            cout << "从目标点云中读取 " << target->size() << " 个点" << endl;
            clock_t e2 = clock();
            whole_time += e2 - s2;
            calaPointCloudCoincide(i, j, source, target, 3);

        }
    }
}



int main()
{

    find_corr();
    cout << "time = " << double(whole_time) / CLOCKS_PER_SEC << "s" << endl;  //输出时间（单位：ｓ）

    system("pause");

    return (0);
}
