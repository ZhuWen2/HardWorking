#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
//点云重合率计算
void calaPointCloudCoincide(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1)
{
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
    core.setInputSource(cloud_src);
    core.setInputTarget(cloud_target);

    boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //共享所有权的智能指针，以kdtree做索引

    core.determineReciprocalCorrespondences(*cor, para1);   //点之间的最大距离,cor对应索引
    std::cout << "点云原来的数量：" << cloud_src->size() << std::endl;
    std::cout << "重合的点云数： " << cor->size() << std::endl;

    std::cout << "重合率： " << double(cor->size()) / double(cloud_src->size()) * 100 << "%" << std::endl;
}

int
main(int argc, char** argv) {
    // 定义输入和输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    char strfilepath1[256] = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\TargetTransformPnts0.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath1, *cloud_in)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }
    char strfilepath2[256] = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\TargetTransformPnts1.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath2, *cloud_out)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }


    //用gicp
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(cloud_in);
    gicp.setInputTarget(cloud_out);
    gicp.setTransformationEpsilon(1e-15);   // 为终止条件设置最小转换差异
    gicp.setMaxCorrespondenceDistance(0.15);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
    gicp.setEuclideanFitnessEpsilon(0.000001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
    gicp.setMaximumIterations(100);           // 最大迭代次数
    //gicp.setMaximumIterations(max_iter);
    gicp.align(*Final);
    pcl::io::savePLYFileASCII("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\Final_gicp.ply", *Final);
    std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
        gicp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& matrix = gicp.getFinalTransformation();
    std::cout << matrix << std::endl;

    //重合度计算
    float para1 = 0.1;
    calaPointCloudCoincide(Final, cloud_out, para1);

    return (0);
}