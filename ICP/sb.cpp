#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/correspondence_estimation.h> 
#include <boost/shared_ptr.hpp>

//把变换矩阵应用到点云上
int
main(int argc, char** argv) {
    // 定义输入和输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    /*char strfilepath1[256] = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\摄影测量更新前的参考点PT11\\摄影测量更新前的参考点PT1.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath1, *cloud_in)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }*/
    char strfilepath2[256] = "E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\PT9.ply";
    if (-1 == pcl::io::loadPLYFile(strfilepath2, *cloud_in1)) {
        std::cout << "error input!" << std::endl;
        return -1;
    }
   
    Eigen::Matrix<double, 3, 3> R1;
    Eigen::Matrix<double, 3, 3> R11;
    R1 << 0.999999999973, -7.21271058263e-06, -1.29033524779e-06,
        7.21277133729e-06, 0.999999998865, 4.70905914094e-05,
        1.28999559553e-06, -4.7090600715e-05, 0.99999999889;
    R11 << 0.99999952141, -0.000946508897019, 0.000247591598375,
        0.000946608197925, 0.999999471462, -0.00040125810001,
        -0.000247211673151, 0.000401492280209, 0.999999888845;

    Eigen::Matrix<double, 3, 1> T1;
    Eigen::Matrix<double, 3, 1> T11;

    T1 << -0.00162763710873,
        0.0310780959105,
        -8.35861551581e-05;
    T11 << 0.555808663968,
        -0.0994131059012,
        -0.0723773440641;
    Eigen::Matrix<double, 3, 1> xyz;
    Eigen::MatrixXd xyzhh;

    //cloud_out->width = cloud_in->size();
    //cloud_out->height = 1;
    //cloud_out->is_dense = false;
    //cloud_out->points.resize(cloud_out->width * cloud_out->height);
    //for (int i = 0; i < cloud_out->size(); ++i) {
    //    
    //    xyz << cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z;
    //    xyzhh = R1 * xyz + T1;

    //    cloud_out->points[i].x = xyzhh(0, 0);
    //    cloud_out->points[i].y = xyzhh(1, 0);
    //    cloud_out->points[i].z = xyzhh(2, 0);
    //}

    //pcl::io::savePLYFileASCII("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\摄影测量更新前的参考点PT11\\2.ply", *cloud_out);

    Eigen::Matrix<double, 3, 1> xyz1;
    cloud_out->width = cloud_in1->size();
    cloud_out->height = 1;
    cloud_out->is_dense = false;
    cloud_out->points.resize(cloud_out->width * cloud_out->height);
    for (int i = 0; i < cloud_out->size(); ++i) {

        xyz1 << cloud_in1->points[i].x, cloud_in1->points[i].y, cloud_in1->points[i].z;
        xyzhh = R11 * xyz1+T11;

        cloud_out->points[i].x = xyzhh(0, 0);
        cloud_out->points[i].y = xyzhh(1, 0);
        cloud_out->points[i].z = xyzhh(2, 0);
    }

    pcl::io::savePLYFileASCII("E:\\XTOP\\XTOPwork\\ICP\\测试数据\\test\\07\\9.ply", *cloud_out);
    std::cout << "hh" << std::endl;
    return (0);
}