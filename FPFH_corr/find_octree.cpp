
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>//�����²����˲�
#include <pcl/features/fpfh_omp.h> //fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/correspondence_estimation.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_svd.h> //�������ֽ����任����
#include <pcl/registration/ia_ransac.h>//sac_ia�㷨
#include <fstream>
#include <time.h>
#include <pcl/octree/octree_search.h>

clock_t whole_time = 0;

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointXYZ point;

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



void getOverlappedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr overlapped_cloud2)
{
    double resolution = 7;
    pcl::octree::OctreePointCloudSearch<point> octree(resolution);
    octree.setInputCloud(cloud1);
    octree.addPointsFromInputCloud();

    //pcl::PointCloud<point> overlapped_2;
    for (size_t i = 0; i < cloud2->size(); ++i)
    {

        std::vector<int> indices;
        octree.voxelSearch(cloud2->points[i], indices);
        pcl::PointCloud<point> cloud_out;
        if (indices.size())
        {
            overlapped_cloud2->push_back(cloud2->points[i]);
        }

    }
}






void delete_wrong_corr(boost::shared_ptr<pcl::Correspondences> cru_correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target) {

    cout << "ɾ��ǰcrude size is:" << cru_correspondences->size() << endl;
    int slow = 0;
    for (int fast = 0; fast < (*cru_correspondences).size(); ++fast) {
        if (
            /*(source->points[(*cru_correspondences)[fast].index_query].x - target->points[(*cru_correspondences)[fast].index_match].x < 1)
            && (source->points[(*cru_correspondences)[fast].index_query].y - target->points[(*cru_correspondences)[fast].index_match].y < 1)
            && (source->points[(*cru_correspondences)[fast].index_query].z - target->points[(*cru_correspondences)[fast].index_match].z < 1)*/
            sqrt(pow((source->points[(*cru_correspondences)[fast].index_query].x - target->points[(*cru_correspondences)[fast].index_match].x), 2)
                + pow((source->points[(*cru_correspondences)[fast].index_query].y - target->points[(*cru_correspondences)[fast].index_match].y), 2)
                + pow((source->points[(*cru_correspondences)[fast].index_query].z - target->points[(*cru_correspondences)[fast].index_match].z), 2)) < 2.3
            ) {
            (*cru_correspondences)[slow].index_query = (*cru_correspondences)[fast].index_query;
            (*cru_correspondences)[slow].index_match = (*cru_correspondences)[fast].index_match;
            ++slow;
        }
    }
    (*cru_correspondences).resize(slow - 1);

    cout << "ɾ����crude size is:" << cru_correspondences->size() << endl;
}


void save_corr(boost::shared_ptr<pcl::Correspondences> cru_correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, int a, int b) {
    string filename1 = "E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\overlap_cloud\\PT" + to_string(a) + "_" + to_string(b) + "_" + to_string(a) + ".txt";
    string filename2 = "E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\overlap_cloud\\PT" + to_string(a) + "_" + to_string(b) + "_" + to_string(b) + ".txt";
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

    ////---------------------------ȥ��Դ���Ƶ�NAN��------------------------
    //vector<int> indices_src; //����ȥ���ĵ������
    //pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
    //cout << "remove *source_cloud nan" << endl;
    //-------------------------Դ�����²����˲�-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vs;
    vs.setLeafSize(0.5, 0.5, 0.5);
    vs.setInputCloud(cloud1);
    pointcloud::Ptr source(new pointcloud);
    vs.filter(*source);
    cout << "down size *cloud1 from " << cloud1->size() << " to " << source->size() << endl;

    //--------------------------ȥ��Ŀ����Ƶ�NAN��--------------------
    vector<int> indices_tgt; //����ȥ���ĵ������
    pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices_tgt);
    cout << "remove *target_cloud nan" << endl;
    //----------------------Ŀ������²����˲�-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vt;
    vt.setLeafSize(0.5, 0.5, 0.5);
    vt.setInputCloud(cloud2);
    pointcloud::Ptr target(new pointcloud);
    vt.filter(*target);
    cout << "down size *target_cloud from " << cloud2->size() << " to " << target->size() << endl;

    clock_t s4 = clock();
    //---------------����Դ���ƺ�Ŀ����Ƶ�FPFH------------------------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    //fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);


    pointnormal::Ptr normals1(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(source);
    n.setNumberOfThreads(8);//����openMP���߳���
    n.setSearchMethod(tree);
    n.setKSearch(14);
    n.compute(*normals1);

    //------------------FPFH����-------------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    f.setNumberOfThreads(8); //ָ��8�˼���
    f.setInputCloud(source);
    f.setInputNormals(normals1);
    f.setSearchMethod(tree);
    f.setKSearch(40);
    f.compute(*fpfh);

    fpfhFeature::Ptr source_fpfh = fpfh;



    //fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);

    pointnormal::Ptr normals2(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n1;
    n1.setInputCloud(target);
    n1.setNumberOfThreads(8);//����openMP���߳���
    n1.setSearchMethod(tree);
    n1.setKSearch(14);
    n1.compute(*normals2);

    //------------------FPFH����-------------------------------
    fpfhFeature::Ptr fpfh1(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f1;
    f1.setNumberOfThreads(8); //ָ��8�˼���
    f1.setInputCloud(target);
    f1.setInputNormals(normals2);
    f1.setSearchMethod(tree);
    f1.setKSearch(40);
    f1.compute(*fpfh1);
    fpfhFeature::Ptr target_fpfh = fpfh1;

    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    crude_cor_est.setInputSource(source_fpfh);
    crude_cor_est.setInputTarget(target_fpfh);
    //crude_cor_est.determineCorrespondences(*cru_correspondences,50);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences, 4);


    //--------------�ų���Ծ������Ĵ���ƥ���----------------------------
    delete_wrong_corr(cru_correspondences, source, target);
    clock_t e4 = clock();
    whole_time += e4 - s4;
    //��ӡ��Ӧ�����
    for (int i = 0; i < (*cru_correspondences).size(); ++i) {
        cout << sqrt(pow((source->points[(*cru_correspondences)[i].index_query].x - target->points[(*cru_correspondences)[i].index_match].x), 2) + pow((source->points[(*cru_correspondences)[i].index_query].y - target->points[(*cru_correspondences)[i].index_match].y), 2) + pow((source->points[(*cru_correspondences)[i].index_query].z - target->points[(*cru_correspondences)[i].index_match].z), 2)) << " ";
    }


    clock_t s5 = clock();
    //--------------�����ҵ��Ķ�Ӧ��----------------------------
    save_corr(cru_correspondences, source, target, a, b);
    clock_t e5 = clock();
    whole_time += e5 - s5;
   

        //-------------------���ӻ�------------------------------------
        //visualize_pcd(source, target, align);
            //-----------------------------�������ֽ���任����-----------
    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr trans(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>);

    trans->estimateRigidTransformation(*source, *target, *cru_correspondences, Transform);

    cout << "SVD��õı任����Ϊ��\n" << Transform << endl;

    end = clock();
    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;
    pcl::transformPointCloud(*source, *source, Transform);
    //pcl::io::savePCDFile("left_sub_reg_right.pcd", *source, false);


    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("��ʾ����"));
    viewer->setBackgroundColor(0, 0, 0);  //���ñ�����ɫΪ��ɫ
    // ��Ŀ�������ɫ���ӻ� (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    // ��Դ������ɫ���ӻ� (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(source, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(source, input_color, "input cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
    //��Ӧ��ϵ���ӻ�
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


    clock_t s3 = clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlapped1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlapped2(new pcl::PointCloud<pcl::PointXYZ>);

    getOverlappedCloud(cloud_src, cloud_target, overlapped2);
   

    //if (double(overlapped2->size())/ cloud_target->size() < 0.1) {
    //    return 0;
    //}
    getOverlappedCloud(cloud_target, cloud_src, overlapped1);
    if (double(overlapped1->size()) / cloud_src->size() < 0.1) {
        return 0;
    }
    //�ֱ���������Ƶ��ص����ֵĵ��Ʊ�������
   
    pcl::io::savePLYFile("E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\overlap_cloud\\PT" + to_string(m) + "_" + to_string(n) + "_" + to_string(m) + ".ply", *overlapped1);
    pcl::io::savePLYFile("E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\overlap_cloud\\PT" + to_string(m) + "_" + to_string(n) + "_" + to_string(n) + ".ply", *overlapped2);
    //��fpfh�Ҷ�Ӧ��
    clock_t e3 = clock();
    whole_time += e3 - s3;

    find_fpfh(overlapped1, overlapped2, m, n);
    cout << double(overlapped2->size()) / cloud_target->size() << endl;
    return double(overlapped2->size()) / cloud_target->size();
}


void find_corr() {

    for (int i = 0; i < 9; ++i) {
        clock_t s1 = clock();
        // --------------------����Դ����-----------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
        string fileName1 = "E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\PT" + to_string(i) + ".ply";
        pcl::io::loadPLYFile(fileName1, *source);
        cout << "��Դ�����ж�ȡ " << source->size() << " ����" << endl;
        clock_t e1 = clock();
        whole_time += e1 - s1;
        for (int j = i + 1; j < 10; ++j) {

            clock_t s2 = clock();
            // -------------------����Ŀ�����----------------------
            pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
            string fileName2 = "E:\\XTOP\\XTOPwork\\ICP\\��������\\test\\07\\PT" + to_string(j) + ".ply";
            pcl::io::loadPLYFile(fileName2, *target);
            cout << "��Ŀ������ж�ȡ " << target->size() << " ����" << endl;
            clock_t e2 = clock();
            whole_time += e2 - s2;
            calaPointCloudCoincide(i, j, source, target, 3);

        }
    }
}



int main()
{

    find_corr();
    cout << "time = " << double(whole_time) / CLOCKS_PER_SEC << "s" << endl;  //���ʱ�䣨��λ����

    system("pause");

    return (0);
}
