//此功能是水下项目中机械臂位姿感知




//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/registration/correspondence_estimation.h> 
//#include <boost/shared_ptr.hpp>
//
//using namespace std;
////把变换矩阵应用到点云上
//int
//main(int argc, char** argv) {
//    //其实下面这段用不上，这是由机械臂的欧拉角的变化和位置的变化求变换矩阵的
//    //其实只要根据点云的RT求变换矩阵，然后应用到机械臂末端第一个位置，就得到了变换后的位置。然后这个位置与真实位置比较距离就得到偏差了。
//
//    ////由欧拉角获得旋转矩阵
//    //    float init_roll = -83.6237, init_pitch = -0.5522, init_yaw = -174.8376;
//
//    //    Eigen::AngleAxisd init_rotation_x(DEG2RAD(init_roll), Eigen::Vector3d::UnitX());
//    //    Eigen::AngleAxisd init_rotation_y(DEG2RAD(init_pitch), Eigen::Vector3d::UnitY());
//    //    Eigen::AngleAxisd init_rotation_z(DEG2RAD(init_yaw), Eigen::Vector3d::UnitZ());
//
//    //    Eigen::Matrix3d R_M;
//    //    R_M = init_rotation_z * init_rotation_y * init_rotation_x;
//    //    std::cout << "R_M: " << std::endl << R_M << std::endl;
//    //    
//    ////求变换矩阵
//    //    Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity();
//    //    rotation_matrix1 = R_M;
//    //    Eigen::Vector3d t1;
//    //    t1 << -0.0121, -0.4635, 0.4649;
//
//    //    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
//    //    T1 = Eigen::Isometry3d::Identity();
//    //    T1.rotate(rotation_matrix1);
//    //    T1.pretranslate(t1);
//    //    cout << "T1 from r,t:\n" << T1.matrix() << endl;
//
//    
//
//    return (0);
//}





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
#include <pcl/filters/normal_space.h>

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

    //------------------------------------------------首先点云粗配准和精配准得到变换矩阵-----------------------------//
    pointcloud::Ptr source_cloud(new pointcloud);
    pointcloud::Ptr target_cloud(new pointcloud);
    pointcloud::Ptr source(new pointcloud);
    pointcloud::Ptr target(new pointcloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pointnormal::Ptr normals(new pointnormal);
    pointnormal::Ptr normalss(new pointnormal);
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pointnormal::Ptr normals1(new pointnormal);
    pointnormal::Ptr normalss1(new pointnormal);
    fpfhFeature::Ptr fpfh1(new fpfhFeature);
    pointcloud::Ptr align(new pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nn;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n1;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nn1;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f1;


    

        clock_t start, end, time;
        start = clock();


        string fileName1, fileName2, fileName3;
        fileName3 = "C:\\Users\\123456\\Desktop\\测试工件\\机械臂扫描\\89.ply";
       
        fileName1 = "C:\\Users\\123456\\Desktop\\测试工件\\机械臂扫描\\8.ply";
        fileName2 = "C:\\Users\\123456\\Desktop\\测试工件\\机械臂扫描\\9.ply";
      



        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName1, *source_cloud);
        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName2, *target_cloud);
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
        n.setInputCloud(source_cloud);
        n.setNumberOfThreads(8);//设置openMP的线程数
        n.setSearchMethod(tree);
        n.setKSearch(15);
        n.compute(*normals);

        // 创建法向空间采样（模板）类对象
        pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
        // 设置xyz三个法向空间的分类组数，此处设置为一致，根据具体场景可以调整
        const int kBinNum = 16;
        nss.setBins(kBinNum, kBinNum, kBinNum);
        // 如果传入的是有序点云，此处可以尝试设置为true
        nss.setKeepOrganized(false);
        // 设置随机种子，这样可以保证同样的输入可以得到同样的结果，便于debug分析
        nss.setSeed(0);   // random seed
        // 传入待采样的点云数据
        nss.setInputCloud(source_cloud);
        // 传入用于采样分析的法线数据，需与传入点云数据一一对应
        nss.setNormals(normals);
        // 设置采样总数，即目标点云的总数据量
        const float kSampleRatio = 0.4f;
        nss.setSample(source_cloud->size() * kSampleRatio);
        // 执行采样并带出采样结果
        nss.filter(*source);
        pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\工件MLS降1.ply", *source);





        ////--------------------------去除目标点云的NAN点--------------------
        vector<int> indices_tgt; //保存去除的点的索引
        pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
        cout << "remove *target_cloud nan" << endl;
        //----------------------目标点云下采样滤波-------------------------
        //pcl::VoxelGrid<pcl::PointXYZ> vt;
        /*vt.setLeafSize(2, 2, 2);
        vt.setInputCloud(target_cloud);

        vt.filter(*target);
        pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\降采样.ply", *target);
        cout << "down size *target_cloud from " << target_cloud->size() << " to " << target->size() << endl;*/
        //pcl::RandomSample<pcl::PointXYZ> vs1;    //创建滤波器对象
        //vs1.setInputCloud(target_cloud);                //设置待滤波点云
        //vs1.setSample(32000);                    //设置下采样点云的点数
        ////vs.setSeed(1);                        //设置随机函数种子点
        //vs1.filter(*target);                    //执行下采样滤波，保存滤波结果于cloud_sub
        //pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\降采样.ply", *target);

        n1.setInputCloud(target_cloud);
        n1.setNumberOfThreads(8);//设置openMP的线程数
        n1.setSearchMethod(tree);
        n1.setKSearch(15);
        n1.compute(*normals1);
        // 创建法向空间采样（模板）类对象
        pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss1;
        // 设置xyz三个法向空间的分类组数，此处设置为一致，根据具体场景可以调整

        nss1.setBins(kBinNum, kBinNum, kBinNum);
        // 如果传入的是有序点云，此处可以尝试设置为true
        nss1.setKeepOrganized(false);
        // 设置随机种子，这样可以保证同样的输入可以得到同样的结果，便于debug分析
        nss1.setSeed(0);   // random seed
        // 传入待采样的点云数据
        nss1.setInputCloud(target_cloud);
        // 传入用于采样分析的法线数据，需与传入点云数据一一对应
        nss1.setNormals(normals1);
        // 设置采样总数，即目标点云的总数据量

        nss1.setSample(target_cloud->size() * kSampleRatio);
        // 执行采样并带出采样结果
        nss1.filter(*target);
        pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\工件MLS降2.ply", *target);
        //---------------计算源点云和目标点云的FPFH------------------------

        //fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
        nn.setInputCloud(source);
        nn.setNumberOfThreads(8);//设置openMP的线程数
        nn.setSearchMethod(tree);
        nn.setKSearch(50);
        nn.compute(*normalss);

        //------------------FPFH估计-------------------------------


        f.setNumberOfThreads(8); //指定8核计算
        f.setInputCloud(source);
        f.setInputNormals(normalss);
        f.setSearchMethod(tree);
        f.setKSearch(50);
        f.compute(*fpfh);

        //fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);
        nn1.setInputCloud(target);
        nn1.setNumberOfThreads(8);//设置openMP的线程数
        nn1.setSearchMethod(tree);
        nn1.setKSearch(50);
        nn1.compute(*normalss1);

        //------------------FPFH估计-------------------------------


        f1.setNumberOfThreads(8); //指定8核计算
        f1.setInputCloud(target);
        f1.setInputNormals(normalss1);
        f1.setSearchMethod(tree);
        f1.setKSearch(50);
        f1.compute(*fpfh1);


        //--------------采样一致性SAC_IA初始配准----------------------------
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
        sac_ia.setInputSource(source);
        sac_ia.setSourceFeatures(fpfh);
        sac_ia.setInputTarget(target);
        sac_ia.setTargetFeatures(fpfh1);
        sac_ia.setMinSampleDistance(3);//设置样本之间的最小距离
        //setNumberOfSamples设置3个就够，设置多了反倒配不准。
        sac_ia.setNumberOfSamples(3);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
        //sac_ia.setCorrespondenceRandomness(20); //在选择随机特征对应时，设置要使用的邻居的数量;
        //也就是计算协方差时选择的近邻点个数，该值越大，协防差越精确，但是计算效率越低.(可省)
        //sac_ia.setErrorFunction();//这个调用是可选的
        sac_ia.setMaximumIterations(100);
        sac_ia.align(*align);
        end = clock();
        pcl::transformPointCloud(*source_cloud, *source_cloud, sac_ia.getFinalTransformation());

        pcl::io::savePLYFile(fileName3, *align);
        cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << "s" << endl;
        cout << "\nSAC_IA has converged, score is " << sac_ia.getFitnessScore() << endl;
        cout << "变换矩阵：\n" << sac_ia.getFinalTransformation() << endl;
        //-------------------可视化------------------------------------
         visualize_pcd(source_cloud, target_cloud, align);






        //----------------------------------icp----------------------------------------
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);
        icp.setTransformationEpsilon(1e-15);   // 为终止条件设置最小转换差异
        icp.setMaxCorrespondenceDistance(8);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
        icp.setEuclideanFitnessEpsilon(0.0001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
        icp.setMaximumIterations(100);           // 最大迭代次数
        // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
        // 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
        // （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
        // 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。

        icp.align(*Final);
        //*Final += *target_cloud;
        pcl::transformPointCloud(*source_cloud, *source_cloud, icp.getFinalTransformation());
        //*source_cloud += *target_cloud;
        pcl::io::savePLYFileASCII(fileName3, *source_cloud);
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << std::endl;
        const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& matrix = icp.getFinalTransformation();
        std::cout << matrix << std::endl;
        visualize_pcd(source_cloud, target_cloud, Final);
        //下面根据粗精配准的变换矩阵求变换后的点位置
        Eigen::Matrix4f T_matrix1 = sac_ia.getFinalTransformation();
        Eigen::Matrix4f T_matrix2 = icp.getFinalTransformation();
        
       /* Eigen::Matrix4f T = T_matrix2 * T_matrix1;
        std::cout << T << std::endl;*/

    //    //p根据手眼标定矩阵来的，p是在末端坐标系观察相机坐标系原点的坐标
    //    Eigen::Vector4f p ,pc;
    //    p << -95.8984, 79.1573, 141.0851,1;
    //    
    ////由欧拉角获得旋转矩阵
    //    float init_roll = -76.744, init_pitch = 21.9, init_yaw = 163.487;
    //    //float init_roll = 163.487, init_pitch = -76.744, init_yaw = 21.9;
    //    //float init_pitch = 163.487, init_yaw = -76.744, init_roll = 21.9;
    //    Eigen::AngleAxisf init_rotation_x(DEG2RAD(init_roll), Eigen::Vector3f::UnitX());
    //    Eigen::AngleAxisf init_rotation_y(DEG2RAD(init_pitch), Eigen::Vector3f::UnitY());
    //    Eigen::AngleAxisf init_rotation_z(DEG2RAD(init_yaw), Eigen::Vector3f::UnitZ());

    //    Eigen::Matrix3f R_M;
    //    R_M = init_rotation_z * init_rotation_y * init_rotation_x;
    //    std::cout << "R_M: " << std::endl << R_M << std::endl;
    //    
    ////求变换矩阵
    //    Eigen::Matrix3f rotation_matrix1 = Eigen::Matrix3f::Identity();
    //    rotation_matrix1 = R_M;
    //    Eigen::Vector3f t1;
    //    t1 << -148.7, -350, 583.1;//这个输入末端位置
    //    
    //    Eigen::Isometry3f T1 = Eigen::Isometry3f::Identity();
    //    T1 = Eigen::Isometry3f::Identity();
    //    T1.rotate(rotation_matrix1);
    //    T1.pretranslate(t1);
    //    cout << "T1 from r,t:\n" << T1.matrix() << endl;
    //    
    //   
    //    pc = T1.matrix() * p;
    //    std::cout << "左相机变换前位置：" << pc << std::endl;
    //    //计算左相机移动后位置
    //    Eigen::Vector4f pc2;
    //    pc2 = T_matrix2 * T_matrix1 * pc;
    //    std::cout << "左相机变换后位置：" << pc2 << std::endl;


    //    //下面计算第二个位姿的真实左相机位置
    //     //由欧拉角获得旋转矩阵
    //    float init_roll2 = -73.785, init_pitch2 = -10.789, init_yaw2 = -148.4167;
    //    //float init_roll2 = -148.4167, init_pitch2 = -73.785, init_yaw2 = -10.789;
    //    //float init_pitch2 = -148.4167, init_yaw2 = -73.785, init_roll2 = -10.789;
    //    Eigen::AngleAxisf init_rotation_x2(DEG2RAD(init_roll2), Eigen::Vector3f::UnitX());
    //    Eigen::AngleAxisf init_rotation_y2(DEG2RAD(init_pitch2), Eigen::Vector3f::UnitY());
    //    Eigen::AngleAxisf init_rotation_z2(DEG2RAD(init_yaw2), Eigen::Vector3f::UnitZ());

    //    Eigen::Matrix3f R_M2;
    //    R_M = init_rotation_z2 * init_rotation_y2 * init_rotation_x2;
    //    std::cout << "R_M2: " << std::endl << R_M2 << std::endl;

    //    //求变换矩阵
    //    Eigen::Matrix3f rotation_matrix2 = Eigen::Matrix3f::Identity();
    //    rotation_matrix2 = R_M2;
    //    Eigen::Vector3f t2;
    //    t2 << -31.19, -378.7, 640.9;//这个输入末端位置

    //    Eigen::Isometry3f T2 = Eigen::Isometry3f::Identity();
    //    T2 = Eigen::Isometry3f::Identity();
    //    T2.rotate(rotation_matrix2);
    //    T2.pretranslate(t2);
    //    cout << "T2 from r,t:\n" << T2.matrix() << endl;

    //    Eigen::Vector4f pc3;
    //    pc3 = T2.matrix() * p;
    //    std::cout << "左相机变换后真实位置：" << pc3 << std::endl;
    //    double dis = sqrt(pow((pc3(0, 0) - pc2(0, 0)), 2) + pow((pc3(1, 0) - pc2(1, 0)), 2) + pow((pc3(2, 0) - pc2(2, 0)), 2));
    //    std::cout << "两点距离：" << dis << std::endl;



//------------------------------------------------然后验证两个点位置误差-----------------------------//
        //由于刚才算的是第一帧点云向第二帧的变换矩阵，所以它的逆矩阵才是相机位置变化的变换矩阵
        //首先得到了S1坐标系下的S2原点坐标p
        Eigen::Matrix4f T = T_matrix2 * T_matrix1;
        T = T.inverse();
        float x1 = T(0, 3), y1 = T(1, 3), z1 = T(2, 3);
        std::cout << x1 << " " << y1 << "" << z1 << std::endl;
        std::cout << T << std::endl;

        Eigen::Vector4f p;
        p << x1, y1, z1,1;



        //第一个变换，从S到E坐标系
        Eigen::Isometry3f T1 = Eigen::Isometry3f::Identity();
        Eigen::Matrix3f rotation_matrix1 = Eigen::Matrix3f::Identity();
        rotation_matrix1 << 0.9731, -0.0006, -0.23,
            -0.23, 0.0059, -0.9731,
            0.002, 0.9999, 0.0056;
        Eigen::Vector3f t1;
        t1 << -95.8984, 79.1573, 141.0851;

        T1 = Eigen::Isometry3f::Identity();
        T1.rotate(rotation_matrix1);
        T1.pretranslate(t1);
        std::cout <<"T1.matrix():" << std::endl << T1.matrix() << std::endl;

        //第二个变换，从E到W坐标系
        //由欧拉角获得旋转矩阵
        float init_roll = -76.744, init_pitch = 21.9, init_yaw = 163.487;//需要输入
      
        Eigen::AngleAxisf init_rotation_x(DEG2RAD(init_roll), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(DEG2RAD(init_pitch), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(DEG2RAD(init_yaw), Eigen::Vector3f::UnitZ());

        Eigen::Matrix3f R_M;
        R_M = init_rotation_z * init_rotation_y * init_rotation_x;
        std::cout << "R_M: " << std::endl << R_M << std::endl;
        
        //求变换矩阵
        Eigen::Matrix3f rotation_matrix2 = Eigen::Matrix3f::Identity();
        rotation_matrix2 = R_M;
        Eigen::Vector3f t2;
        t2 << -148.7, -350, 583.1;////需要输入这个输入末端位置
        
        Eigen::Isometry3f T2 = Eigen::Isometry3f::Identity();
        T2 = Eigen::Isometry3f::Identity();
        T2.rotate(rotation_matrix2);
        T2.pretranslate(t2);
        cout << "T2 from r,t:\n" << T2.matrix() << endl;

        //得到了从第一个路径算出的S2原点的世界坐标系
        p = T2.matrix() * T1.matrix() * p;
        cout << "p:  " << p << endl;




        //下面从第二个路径计算
        Eigen::Vector4f p2;
        p2 << -95.8984, 79.1573, 141.0851, 1; 
        //p2 << 111.27, -141.625, 54.187, 1;
        //由欧拉角获得旋转矩阵
        float init_roll22 = -73.785, init_pitch22 = -10.789, init_yaw22 = -148.4167;//需要输入
       
        Eigen::AngleAxisf init_rotation_x22(DEG2RAD(init_roll22), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y22(DEG2RAD(init_pitch22), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z22(DEG2RAD(init_yaw22), Eigen::Vector3f::UnitZ());

        Eigen::Matrix3f R_M22;
        R_M22 = init_rotation_z22 * init_rotation_y22 * init_rotation_x22;
        std::cout << "R_M22: " << std::endl << R_M22 << std::endl;

        //求变换矩阵
        Eigen::Matrix3f rotation_matrix22= Eigen::Matrix3f::Identity();
        rotation_matrix22 = R_M22;
        Eigen::Vector3f t22;
        t22 << -31.19, -378.7, 640.9;////需要输入这个输入末端位置

        Eigen::Isometry3f T22 = Eigen::Isometry3f::Identity();
        T22 = Eigen::Isometry3f::Identity();
        T22.rotate(rotation_matrix22);
        T22.pretranslate(t22);
        cout << "T22 from r,t:\n" << T22.matrix() << endl;

        p2 = T22.matrix()* p2;
        cout << "p2:  " << p2 << endl;
        double dis = sqrt(pow((p(0, 0) - p2(0, 0)), 2) + pow((p(1, 0) - p2(1, 0)), 2) + pow((p(2, 0) - p2(2, 0)), 2));
        std::cout << "两点距离：" << dis << std::endl;


        ////输入机械臂末端到左相机的变换矩阵

        //Eigen::Isometry3f T1 = Eigen::Isometry3f::Identity();
        //Eigen::Matrix3f rotation_matrix1 = Eigen::Matrix3f::Identity();
        //rotation_matrix1 << 0.9731, -0.0006, -0.23,
        //    -0.23, 0.0059, -0.9731,
        //    0.002, 0.9999, 0.0056;
        //Eigen::Vector3f t1;
        //t1 << -95.8984, 79.1573, 141.0851;

        //T1 = Eigen::Isometry3f::Identity();
        //T1.rotate(rotation_matrix1);
        //T1.pretranslate(t1);
        //std::cout <<"T1.matrix():" << std::endl << T1.matrix() << std::endl;

        ////得到左相机位置坐标
        //Eigen::Vector4f pc;
        //pc = T1.matrix() * p;
        //std::cout <<"左相机变换前位置：" << pc << std::endl;
        ////计算左相机移动后位置
        //Eigen::Vector4f p1;
        //p1 = T_matrix2 * T_matrix1 * pc;
        //std::cout << "左相机变换后位置：" << p1 << std::endl;
        //
        ////计算与真实点距离
        //Eigen::Vector4f p2;
        //p2 << -31, -378.7, 640.9,1;//待填入

        //Eigen::Isometry3f T2 = Eigen::Isometry3f::Identity();
        //Eigen::Matrix3f rotation_matrix2 = Eigen::Matrix3f::Identity();
        //rotation_matrix2 << 0.9731, -0.0006, -0.23,
        //    -0.23, 0.0059, -0.9731,
        //    0.002, 0.9999, 0.0056;
        //Eigen::Vector3f t2;
        //t2 << -95.8984, 79.1573, 141.0851;

        //T2 = Eigen::Isometry3f::Identity();
        //T2.rotate(rotation_matrix1);
        //T2.pretranslate(t2);
        //std::cout << "T2.matrix():" << std::endl << T2.matrix() << std::endl;

        ////
        //Eigen::Vector4f pc2;
        //pc2 = T2.matrix() * p2;
        //std::cout << "左相机变换前位置：" << pc << std::endl;
        //double dis = sqrt(pow((p1(0, 0) - pc2(0, 0)), 2) + pow((p1(1, 0) - pc2(1, 0)), 2) + pow((p1(2, 0) - pc2(2, 0)), 2));
        //std::cout << "两点距离：" << dis << std::endl;
    return 0;
}










//Eigen::Isometry3f T1 = Eigen::Isometry3f::Identity();
//Eigen::Matrix3f rotation_matrix1 = Eigen::Matrix3f::Identity();
//rotation_matrix1 << 0.9733, -0.23, 0.00194,
//-0.00066, 0.0059, 1,
//-0.23, -0.9732, 0.0056;
//Eigen::Vector3f t1;
//t1 << 111.27, -141.625, 54.187;