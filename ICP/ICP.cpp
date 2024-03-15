#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/correspondence_estimation.h> 
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

    char strfilepath1[256] = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\摄影测量更新前的参考点PT11\\摄影测量更新前的参考点PT10.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath1, *cloud_in)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }
    char strfilepath2[256] = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\摄影测量更新前的参考点PT11\\摄影测量更新前的参考点PT11.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath2, *cloud_out)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }

    // 随机填充无序点云
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

    //// 在点云上执行简单的刚性变换，将cloud_out中的x平移0.7f米，然后再次输出数据值。
    //for (size_t i = 0; i < cloud_in->points.size(); ++i)
    //    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    //// 打印这些点
    //std::cout << "Transformed " << cloud_in->points.size() << " data points:"
    //    << std::endl;
    //for (size_t i = 0; i < cloud_out->points.size(); ++i)
    //    std::cout << "    " << cloud_out->points[i].x << " " <<
    //    cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

    // 创建IterativeClosestPoint的实例
    // setInputSource将cloud_in作为输入点云
    // setInputTarget将平移后的cloud_out作为目标点云
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setTransformationEpsilon(1e-15);   // 为终止条件设置最小转换差异
    icp.setMaxCorrespondenceDistance(5);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
    icp.setEuclideanFitnessEpsilon(0.0001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
    icp.setMaximumIterations(1);           // 最大迭代次数
    // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
    // 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
    // （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
    // 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*Final);
    pcl::io::savePLYFileASCII("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\Final_icp.ply", *Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& matrix = icp.getFinalTransformation();
    std::cout << matrix << std::endl;

    //重合度计算
    float para1=0.1;
    calaPointCloudCoincide(Final, cloud_out, para1);

   
    return (0);
}