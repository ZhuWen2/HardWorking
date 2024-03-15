#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/ply_io.h>
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

    char strfilepath1[256] = "E:\\XTOP\\XTOPwork\\ICP\\��������\\TargetTransformPnts0.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath1, *cloud_in)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }
    char strfilepath2[256] = "E:\\XTOP\\XTOPwork\\ICP\\��������\\TargetTransformPnts1.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath2, *cloud_out)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }


    //��gicp
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(cloud_in);
    gicp.setInputTarget(cloud_out);
    gicp.setTransformationEpsilon(1e-15);   // Ϊ��ֹ����������Сת������
    gicp.setMaxCorrespondenceDistance(0.15);  // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
    gicp.setEuclideanFitnessEpsilon(0.000001);  // �������������Ǿ�������С����ֵ�� ֹͣ������
    gicp.setMaximumIterations(100);           // ����������
    //gicp.setMaximumIterations(max_iter);
    gicp.align(*Final);
    pcl::io::savePLYFileASCII("E:\\XTOP\\XTOPwork\\ICP\\��������\\Final_gicp.ply", *Final);
    std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
        gicp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& matrix = gicp.getFinalTransformation();
    std::cout << matrix << std::endl;

    //�غ϶ȼ���
    float para1 = 0.1;
    calaPointCloudCoincide(Final, cloud_out, para1);

    return (0);
}