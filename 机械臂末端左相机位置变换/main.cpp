//�˹�����ˮ����Ŀ�л�е��λ�˸�֪




//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/registration/correspondence_estimation.h> 
//#include <boost/shared_ptr.hpp>
//
//using namespace std;
////�ѱ任����Ӧ�õ�������
//int
//main(int argc, char** argv) {
//    //��ʵ��������ò��ϣ������ɻ�е�۵�ŷ���ǵı仯��λ�õı仯��任�����
//    //��ʵֻҪ���ݵ��Ƶ�RT��任����Ȼ��Ӧ�õ���е��ĩ�˵�һ��λ�ã��͵õ��˱任���λ�á�Ȼ�����λ������ʵλ�ñȽϾ���͵õ�ƫ���ˡ�
//
//    ////��ŷ���ǻ����ת����
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
//    ////��任����
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
#include <pcl/filters/normal_space.h>

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
        fileName3 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\source1.ply";
       
        fileName1 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\�ں�ǰ��1.ply";
        fileName2 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\�ں�ǰ����1.ply";
      



        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName1, *source_cloud);
        pcl::io::loadPLYFile<pcl::PointXYZ>(fileName2, *target_cloud);
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
        n.setInputCloud(source_cloud);
        n.setNumberOfThreads(8);//����openMP���߳���
        n.setSearchMethod(tree);
        n.setKSearch(15);
        n.compute(*normals);

        // ��������ռ������ģ�壩�����
        pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
        // ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���
        const int kBinNum = 16;
        nss.setBins(kBinNum, kBinNum, kBinNum);
        // ����������������ƣ��˴����Գ�������Ϊtrue
        nss.setKeepOrganized(false);
        // ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
        nss.setSeed(0);   // random seed
        // ����������ĵ�������
        nss.setInputCloud(source_cloud);
        // �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
        nss.setNormals(normals);
        // ���ò�����������Ŀ����Ƶ���������
        const float kSampleRatio = 0.4f;
        nss.setSample(source_cloud->size() * kSampleRatio);
        // ִ�в����������������
        nss.filter(*source);
        pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����MLS��1.ply", *source);





        ////--------------------------ȥ��Ŀ����Ƶ�NAN��--------------------
        vector<int> indices_tgt; //����ȥ���ĵ������
        pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
        cout << "remove *target_cloud nan" << endl;
        //----------------------Ŀ������²����˲�-------------------------
        //pcl::VoxelGrid<pcl::PointXYZ> vt;
        /*vt.setLeafSize(2, 2, 2);
        vt.setInputCloud(target_cloud);

        vt.filter(*target);
        pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\������.ply", *target);
        cout << "down size *target_cloud from " << target_cloud->size() << " to " << target->size() << endl;*/
        //pcl::RandomSample<pcl::PointXYZ> vs1;    //�����˲�������
        //vs1.setInputCloud(target_cloud);                //���ô��˲�����
        //vs1.setSample(32000);                    //�����²������Ƶĵ���
        ////vs.setSeed(1);                        //��������������ӵ�
        //vs1.filter(*target);                    //ִ���²����˲��������˲������cloud_sub
        //pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\������.ply", *target);

        n1.setInputCloud(target_cloud);
        n1.setNumberOfThreads(8);//����openMP���߳���
        n1.setSearchMethod(tree);
        n1.setKSearch(15);
        n1.compute(*normals1);
        // ��������ռ������ģ�壩�����
        pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss1;
        // ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���

        nss1.setBins(kBinNum, kBinNum, kBinNum);
        // ����������������ƣ��˴����Գ�������Ϊtrue
        nss1.setKeepOrganized(false);
        // ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
        nss1.setSeed(0);   // random seed
        // ����������ĵ�������
        nss1.setInputCloud(target_cloud);
        // �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
        nss1.setNormals(normals1);
        // ���ò�����������Ŀ����Ƶ���������

        nss1.setSample(target_cloud->size() * kSampleRatio);
        // ִ�в����������������
        nss1.filter(*target);
        pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����MLS��2.ply", *target);
        //---------------����Դ���ƺ�Ŀ����Ƶ�FPFH------------------------

        //fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
        nn.setInputCloud(source);
        nn.setNumberOfThreads(8);//����openMP���߳���
        nn.setSearchMethod(tree);
        nn.setKSearch(50);
        nn.compute(*normalss);

        //------------------FPFH����-------------------------------


        f.setNumberOfThreads(8); //ָ��8�˼���
        f.setInputCloud(source);
        f.setInputNormals(normalss);
        f.setSearchMethod(tree);
        f.setKSearch(50);
        f.compute(*fpfh);

        //fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);
        nn1.setInputCloud(target);
        nn1.setNumberOfThreads(8);//����openMP���߳���
        nn1.setSearchMethod(tree);
        nn1.setKSearch(50);
        nn1.compute(*normalss1);

        //------------------FPFH����-------------------------------


        f1.setNumberOfThreads(8); //ָ��8�˼���
        f1.setInputCloud(target);
        f1.setInputNormals(normalss1);
        f1.setSearchMethod(tree);
        f1.setKSearch(50);
        f1.compute(*fpfh1);


        //--------------����һ����SAC_IA��ʼ��׼----------------------------
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
        sac_ia.setInputSource(source);
        sac_ia.setSourceFeatures(fpfh);
        sac_ia.setInputTarget(target);
        sac_ia.setTargetFeatures(fpfh1);
        sac_ia.setMinSampleDistance(3);//��������֮�����С����
        //setNumberOfSamples����3���͹������ö��˷����䲻׼��
        sac_ia.setNumberOfSamples(3);  //����ÿ�ε���������ʹ�õ�������������ʡ��,�ɽ�ʡʱ��
        //sac_ia.setCorrespondenceRandomness(20); //��ѡ�����������Ӧʱ������Ҫʹ�õ��ھӵ�����;
        //Ҳ���Ǽ���Э����ʱѡ��Ľ��ڵ��������ֵԽ��Э����Խ��ȷ�����Ǽ���Ч��Խ��.(��ʡ)
        //sac_ia.setErrorFunction();//��������ǿ�ѡ��
        sac_ia.setMaximumIterations(100);
        sac_ia.align(*align);
        end = clock();
        pcl::transformPointCloud(*source_cloud, *source_cloud, sac_ia.getFinalTransformation());

        //pcl::io::savePLYFile("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\zhua.ply", *align);
        cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << "s" << endl;
        cout << "\nSAC_IA has converged, score is " << sac_ia.getFitnessScore() << endl;
        cout << "�任����\n" << sac_ia.getFinalTransformation() << endl;
        //-------------------���ӻ�------------------------------------
        // visualize_pcd(source_cloud, target_cloud, align);






        //----------------------------------icp----------------------------------------
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);
        icp.setTransformationEpsilon(1e-15);   // Ϊ��ֹ����������Сת������
        icp.setMaxCorrespondenceDistance(8);  // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
        icp.setEuclideanFitnessEpsilon(0.0001);  // �������������Ǿ�������С����ֵ�� ֹͣ������
        icp.setMaximumIterations(100);           // ����������
        // ����һ�� pcl::PointCloud<pcl::PointXYZ>ʵ�� Final ����,�洢��׼�任���Դ����,
        // Ӧ�� ICP �㷨��, IterativeClosestPoint �ܹ����������Ƽ�,�������������ƥ����ȷ�Ļ�
        // ������������һ��Ӧ��ĳ�ָ���任���Ϳ��Եõ�������ͬһ����ϵ����ͬ�ĵ��ƣ�,��ô icp. hasConverged()= 1 (true),
        // Ȼ���������ձ任�����ƥ������ͱ任�������Ϣ��

        icp.align(*Final);
        *Final += *target_cloud;
        pcl::transformPointCloud(*source_cloud, *source_cloud, icp.getFinalTransformation());
        //*source_cloud += *target_cloud;
        pcl::io::savePLYFileASCII(fileName3, *source_cloud);
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << std::endl;
        const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& matrix = icp.getFinalTransformation();
        std::cout << matrix << std::endl;

        //������ݴ־���׼�ı任������任��ĵ�λ��
        Eigen::Matrix4f T_matrix1 = sac_ia.getFinalTransformation();
        Eigen::Matrix4f T_matrix2 = icp.getFinalTransformation();
        
       /* Eigen::Matrix4f T = T_matrix2 * T_matrix1;
        std::cout << T << std::endl;*/

        //p�ǻ�е��ĩ�˵�λ�ã��������������λ��
        Eigen::Vector4f p;
        p << -12.1, -463.5, 464.9, 1;

        //�����е��ĩ�˵�������ı任����

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

        //�õ������λ������
        Eigen::Vector4f pc;
        pc = T1.matrix() * p;
        std::cout <<"������任ǰλ�ã�" << pc << std::endl;
        //����������ƶ���λ��
        Eigen::Vector4f p1;
        p1 = T_matrix2 * T_matrix1 * pc;
        std::cout << "������任��λ�ã�" << p1 << std::endl;
        
        //��������ʵ�����
        Eigen::Vector3f p2;
        p2 << -95.8984, 79.1573, 141.0851;//������
        double dis = sqrt(pow((p1(0, 0) - p2(0, 0)), 2) + pow((p1(1, 0) - p2(1, 0)), 2) + pow((p1(2, 0) - p2(2, 0)), 2));
        std::cout << "������룺" << dis << std::endl;
    return 0;
}
