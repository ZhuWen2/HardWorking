#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/correspondence_estimation.h> 
#include <boost/shared_ptr.hpp>
//�����غ��ʼ���
void calaPointCloudCoincide(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1)
{
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
    core.setInputSource(cloud_src);
    core.setInputTarget(cloud_target);

    boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //��������Ȩ������ָ�룬��kdtree������

    core.determineReciprocalCorrespondences(*cor, para1);   //��֮���������,cor��Ӧ����
    std::cout << "����ԭ����������" << cloud_src->size() << std::endl;
    std::cout << "�غϵĵ������� " << cor->size() << std::endl;

    std::cout << "�غ��ʣ� " << double(cor->size()) / double(cloud_src->size()) * 100 << "%" << std::endl;
}

int
main(int argc, char** argv) {
    // ����������������
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    char strfilepath1[256] = "E:\\XTOP\\XTOPwork\\ICP\\��������\\��Ӱ��������ǰ�Ĳο���PT11\\��Ӱ��������ǰ�Ĳο���PT10.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath1, *cloud_in)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }
    char strfilepath2[256] = "E:\\XTOP\\XTOPwork\\ICP\\��������\\��Ӱ��������ǰ�Ĳο���PT11\\��Ӱ��������ǰ�Ĳο���PT11.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath2, *cloud_out)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }

    // �������������
    //cloud_in->width = 5;
    //cloud_in->height = 1;
    //cloud_in->is_dense = false;
    //cloud_in->points.resize(cloud_in->width * cloud_in->height);
    //for (size_t i = 0; i < cloud_in->points.size(); ++i) {
    //    cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    //    cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    //    cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    //}
    //std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
    //    << std::endl;
    //for (size_t i = 0; i < cloud_in->points.size(); ++i)
    //    std::cout << "    " <<
    //    cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
    //    cloud_in->points[i].z << std::endl;
    //*cloud_out = *cloud_in;
    //std::cout << "size:" << cloud_out->points.size() << std::endl;

    //// �ڵ�����ִ�м򵥵ĸ��Ա任����cloud_out�е�xƽ��0.7f�ף�Ȼ���ٴ��������ֵ��
    //for (size_t i = 0; i < cloud_in->points.size(); ++i)
    //    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    //// ��ӡ��Щ��
    //std::cout << "Transformed " << cloud_in->points.size() << " data points:"
    //    << std::endl;
    //for (size_t i = 0; i < cloud_out->points.size(); ++i)
    //    std::cout << "    " << cloud_out->points[i].x << " " <<
    //    cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

    // ����IterativeClosestPoint��ʵ��
    // setInputSource��cloud_in��Ϊ�������
    // setInputTarget��ƽ�ƺ��cloud_out��ΪĿ�����
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setTransformationEpsilon(1e-15);   // Ϊ��ֹ����������Сת������
    icp.setMaxCorrespondenceDistance(5);  // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
    icp.setEuclideanFitnessEpsilon(0.0001);  // �������������Ǿ�������С����ֵ�� ֹͣ������
    icp.setMaximumIterations(1);           // ����������
    // ����һ�� pcl::PointCloud<pcl::PointXYZ>ʵ�� Final ����,�洢��׼�任���Դ����,
    // Ӧ�� ICP �㷨��, IterativeClosestPoint �ܹ����������Ƽ�,�������������ƥ����ȷ�Ļ�
    // ������������һ��Ӧ��ĳ�ָ���任���Ϳ��Եõ�������ͬһ����ϵ����ͬ�ĵ��ƣ�,��ô icp. hasConverged()= 1 (true),
    // Ȼ���������ձ任�����ƥ������ͱ任�������Ϣ��
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*Final);
    pcl::io::savePLYFileASCII("E:\\XTOP\\XTOPwork\\ICP\\��������\\Final_icp.ply", *Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& matrix = icp.getFinalTransformation();
    std::cout << matrix << std::endl;

    //�غ϶ȼ���
    float para1=0.1;
    calaPointCloudCoincide(Final, cloud_out, para1);

   
    return (0);
}