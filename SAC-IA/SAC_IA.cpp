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


    for (int i = 1; i < 2; ++i) {
        
        clock_t start, end, time;
        start = clock();
       

        string fileName1, fileName2, fileName3;
        fileName3 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\source" + to_string(i) + ".ply";
        if (i == 1) {
             fileName1 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\�ں�ǰ��" + to_string(i) + ".ply";
             fileName2 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\�ں�ǰ����" + to_string(i) + ".ply";
        }
        else {
             fileName1 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\a" + to_string(i-1) + ".ply";
             fileName2 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\China" + to_string(i+1) + ".ply";
        }
        


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

    }
    
    return 0;
}
