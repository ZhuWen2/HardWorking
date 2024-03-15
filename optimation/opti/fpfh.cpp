#include<iostream>

#include"optimate.h"

#include "ceres\ceres.h"
#include "ceres\rotation.h"
#include"fstream"
#include <Eigen/Core>
#include <Eigen/Dense>


#include <filesystem>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>


double calaPointCloudCoincide(int m, int n, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
	core.setInputSource(cloud_src);
	core.setInputTarget(cloud_target);

	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //��������Ȩ������ָ�룬��kdtree������

	core.determineReciprocalCorrespondences(*cor, para1);   //��֮���������,cor��Ӧ����
	double chonghe = double(cor->size()) / double(cloud_src->size());

	if (chonghe < 0.1) {
		return 0;
	}
	//��ӡ����Ӧ��
	std::string fileName1 = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/P" + std::to_string(m) + "_" + std::to_string(n) + "_" + std::to_string(m) + ".txt";
	std::string fileName2 = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/P" + std::to_string(m) + "_" + std::to_string(n) + "_" + std::to_string(n) + ".txt";
	std::ofstream query_value1;
	query_value1.open(fileName1, std::ios::out);
	std::ofstream query_value2;
	query_value2.open(fileName2, std::ios::out);
	for (int i = 0; i < (*cor).size(); ++i) {
		query_value1 << cloud_src->points[(*cor)[i].index_query].x << " "
			<< cloud_src->points[(*cor)[i].index_query].y << " "
			<< cloud_src->points[(*cor)[i].index_query].z << " "
			<< std::endl;

		query_value2 << cloud_target->points[(*cor)[i].index_match].x << " "
			<< cloud_target->points[(*cor)[i].index_match].y << " "
			<< cloud_target->points[(*cor)[i].index_match].z << " "
			<< std::endl;
	}
	query_value1.close();
	query_value2.close();
	return chonghe;
}

//�Ҷ�Ӧ��
void find_corr() {
	for (int i = 0; i < 9; ++i) {
		// --------------------����Դ����-----------------------
		pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
		std::string fileName1 = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/PT" + std::to_string(i) + ".ply";
		pcl::io::loadPLYFile(fileName1, *source);
		std::cout << "��Դ�����ж�ȡ " << source->size() << " ����" << std::endl;
		for (int j = i + 1; j < 10; ++j) {
			// -------------------����Ŀ�����----------------------
			pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
			std::string fileName2 = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/PT" + std::to_string(j) + ".ply";
			pcl::io::loadPLYFile(fileName2, *target);
			std::cout << "��Ŀ������ж�ȡ " << target->size() << " ����" << std::endl;

			calaPointCloudCoincide(i, j, source, target, 3);

		}

	}
}

//��ȡ�ļ����ļ�����
int read(std::string filename_global_points, std::vector<std::string>& filenames) {
	//std::std::string path = "./test_8_15/ExportData_׷��";  // �滻�����ʵ���ļ�·��
	//std::std::vector<std::std::string> filenames;

	//namespace fs = std::filesystem;
	//for (const auto& entry : fs::directory_iterator(filename_global_points)) {
	//	if (fs::is_regular_file(entry)) {
	//		filenames.push_back(entry.path().filename().string());
	//	}
	//}

	//std::sort(filenames.begin(), filenames.end());

	//for (int i = 0; i < filenames.size(); i++)
	//{
	//	filenames[i] = filename_global_points + "/" + filenames[i];
	//}
	int pointSum = 10;

	for (int i = 0; i < pointSum - 1; i++)
	{
		for (int j = i + 1; j < pointSum; ++j) {
			std::string file1 = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/PT" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(i) + ".txt";
			std::string file2 = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/PT" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(j) + ".txt";


			std::ifstream fin1(file1);
			if (!fin1.is_open()) continue;
			else {
				filenames.push_back(file1);
				filenames.push_back(file2);
			}
		}
	}
	return 0;
}


//��ȡ�ļ����ļ�����
int read2(std::string filename_global_points, std::vector<std::string>& filenames) {
	
	int pointSum = 10;

	for (int i = 0; i < pointSum - 1; i++)
	{
		for (int j = i + 1; j < pointSum; ++j) {
			std::string file1 = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/P" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(i) + ".txt";
			std::string file2 = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/P" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(j) + ".txt";


			std::ifstream fin1(file1);
			if (!fin1.is_open()) continue;
			else {
				filenames.push_back(file1);
				filenames.push_back(file2);
			}
		}
	}
	return 0;
}


//��������Ǽ���FPFH�Ż�֮��Ķ�Ӧ�����
void CalAfterErr() {
	for (int itnum = 0; itnum < 1; ++itnum) {
		std::cout.precision(12);

		std::string folder = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/PT";
		std::vector<std::string> samePointsFile;
		//���Ӧ��
		find_corr();

		read2(folder, samePointsFile);
		for (int i = 0; i < samePointsFile.size(); i++)
		{
			std::cout << samePointsFile[i] << std::endl;
		}

		int Length_Names = samePointsFile.size();
		std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> PointCloudMat(Length_Names / 2, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>());
		int pointCloudNum = 0;//��������

		for (int i = 0; i < Length_Names; i++)
		{
			std::string filePath = samePointsFile[i];
			std::string fileName = filePath.substr(0, filePath.length() - 4);
			fileName = fileName.substr(57, fileName.size());
			std::cout << fileName << std::endl;
			long pos = fileName.find('_', 0);
			int firstIndex = stoi(fileName.substr(0, long(pos)));

			int pos1 = fileName.find('_', long(pos) + 1);
			int secondIndex = stoi(fileName.substr(long(pos) + 1, pos1));

			int betweenIndex = stoi(fileName.substr(long(pos1) + 1, fileName.size()));

			//�ж϶��ٷ�����
			if (secondIndex + 1 > pointCloudNum);
			{
				pointCloudNum = secondIndex + 1;
			}

			//��ȡ��������
			std::ifstream fin1(filePath);
			std::string line1;
			Eigen::Matrix<double, Eigen::Dynamic, 3> onePointCloud;
			std::vector<Eigen::Vector3d> pointCloud1_vec;
			while (std::getline(fin1, line1))
			{
				std::stringstream ss(line1);
				double point_X;
				double point_Y;
				double point_Z;
				ss >> point_X >> point_Y >> point_Z;
				Eigen::Vector3d point(point_X, point_Y, point_Z);

				pointCloud1_vec.push_back(point);
			}
			fin1.close();

			onePointCloud.conservativeResize(pointCloud1_vec.size() + 1, Eigen::NoChange);
			Eigen::Vector3d indexVec(firstIndex, secondIndex, betweenIndex);
			onePointCloud.row(0) = indexVec;
			for (int k = 1; k < pointCloud1_vec.size() + 1; k++)
			{
				onePointCloud.row(k) = pointCloud1_vec[k - 1];
			}

			if (firstIndex == betweenIndex)
			{
				PointCloudMat[i / 2].first = onePointCloud;
			}
			else
			{
				PointCloudMat[i / 2].second = onePointCloud;
			}



			//std::cout << fileName << " " << firstIndex << " " << secondIndex<<" "<< betweenIndex << std::endl;;
		}

		/*for (int i=0;i< PointCloudMat.size();i++)
		{
			Eigen::Vector3d p1= PointCloudMat[i].first.row(0);
			Eigen::Vector3d p2= PointCloudMat[i].second.row(0);
			std::cout << "-------------" << std::endl;
			std::cout << p1 << std::endl << p2 << std::endl;


			Eigen::MatrixXd mat1 = (PointCloudMat[i].first).block(1, 0, (PointCloudMat[i].first).rows() - 1, (PointCloudMat[i].first).cols());
			Eigen::MatrixXd mat2 = (PointCloudMat[i].second).block(1, 0, (PointCloudMat[i].second).rows() - 1, (PointCloudMat[i].second).cols());

			std::cout << "mat1: " << std::endl;
			std::cout << mat1 << std::endl;
			std::cout << "mat2: " << std::endl;
			std::cout << mat2 << std::endl;
		}*/



		//����ÿ�����ƶ�Ӧ��RT
		Eigen::Vector3d* rotationVec = new Eigen::Vector3d[pointCloudNum];
		Eigen::Vector3d* translation = new Eigen::Vector3d[pointCloudNum];
		//��ת������ƽ��������ʼ��
		for (int i = 0; i < pointCloudNum; i++)
		{
			rotationVec[i] = Eigen::Vector3d(0, 0, 0);
			translation[i] = Eigen::Vector3d(0, 0, 0);
		}

		//��¼����ͶӰ���(dx2+dy2+dz2)
		std::vector<double> err_pre;
		std::vector<double> err_cur;



		//�Ż�֮ǰ
		//�������֮��ƽ�����
		Eigen::Vector3d err0(0, 0, 0);
		for (int i = 0; i < PointCloudMat.size(); i++)
		{
			//ȡ����Ӧ�����±�
			int index1 = -1, index2 = -1;
			index1 = (int)(PointCloudMat[i].first).row(0)(2);
			index2 = (int)(PointCloudMat[i].second).row(0)(2);

			//ȡ����Ӧ����
			Eigen::MatrixXd pointCloud_pre, pointCloud_cur;
			pointCloud_pre = (PointCloudMat[i].first).block(1, 0, (PointCloudMat[i].first).rows() - 1, (PointCloudMat[i].first).cols());
			pointCloud_cur = (PointCloudMat[i].second).block(1, 0, (PointCloudMat[i].second).rows() - 1, (PointCloudMat[i].second).cols());

			Eigen::MatrixXd pointMat_pre1 = pointCloud_pre.transpose();
			Eigen::MatrixXd pointMat_cur1 = pointCloud_cur.transpose();

			//ȡ����ǰRT
			Eigen::Vector3d RVec_pre = rotationVec[index1];
			Eigen::Matrix3d R_Mat_pre;
			R_Mat_pre = Eigen::AngleAxisd(RVec_pre.norm(), RVec_pre.normalized());

			Eigen::Vector3d RVec_cur = rotationVec[index2];
			Eigen::Matrix3d R_Mat_cur;
			R_Mat_cur = Eigen::AngleAxisd(RVec_cur.norm(), RVec_cur.normalized());

			Eigen::Vector3d T_Vec_pre = translation[index1];
			Eigen::Vector3d T_Vec_cur = translation[index2];

			Eigen::MatrixXd errMat = R_Mat_cur * pointMat_cur1 - (R_Mat_pre * pointMat_pre1);
			for (int m = 0; m < errMat.cols(); m++)
			{
				errMat(0, m) = errMat(0, m) + T_Vec_cur(0) - T_Vec_pre(0);
				errMat(1, m) = errMat(1, m) + T_Vec_cur(1) - T_Vec_pre(1);
				errMat(2, m) = errMat(2, m) + T_Vec_cur(2) - T_Vec_pre(2);
			}


			Eigen::Vector3d errColumAbsSum(0, 0, 0);

			double err_every = 0;
			for (int m = 0; m < errMat.cols(); m++)
			{
				errColumAbsSum(0) += abs(errMat(0, m));
				errColumAbsSum(1) += abs(errMat(1, m));
				errColumAbsSum(2) += abs(errMat(2, m));


				err_every += sqrt(errMat(0, m) * errMat(0, m) + errMat(1, m) * errMat(1, m) + errMat(2, m) * errMat(2, m));
			}
			err_every = err_every / errMat.cols();
			err_pre.push_back(err_every);

			errColumAbsSum = errColumAbsSum / errMat.cols();

			err0 = err0 + errColumAbsSum;
			std::cout << "--------------------------" << std::endl;
			std::cout << "---------" << "pointCloud " << i + 1 << " to " << "pointCloud " << i << "---------" << std::endl;
			std::cout << "preErr_X: " << errColumAbsSum(0) << " preErr_Y " << errColumAbsSum(1) << " preErr_Z " << errColumAbsSum(2) << std::endl;


		}
		err0 = err0 / (pointCloudNum - 1);

		std::cout << "----------------���е����Ż�ǰx,y,z��Ӧ���ľ���ֵȡƽ��---------------" << std::endl;
		std::cout << err0 << std::endl;
	}
}




int main()
{
	clock_t start = clock();
	for (int itnum = 0; itnum < 1; ++itnum) {
		std::cout.precision(12);

		std::string folder = "E:/XTOP/XTOPwork/optimation/optimation - ����/fpfh_test/PT";
		std::vector<std::string> samePointsFile;
		//���Ӧ��
		//find_corr();

		read(folder, samePointsFile);
		for (int i = 0; i < samePointsFile.size(); i++)
		{
			std::cout << samePointsFile[i] << std::endl;
		}

		int Length_Names = samePointsFile.size();
		std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> PointCloudMat(Length_Names / 2, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>());
		int pointCloudNum = 0;//��������

		for (int i = 0; i < Length_Names; i++)
		{
			std::string filePath = samePointsFile[i];
			std::string fileName = filePath.substr(0, filePath.length() - 4);
			fileName = fileName.substr(58, fileName.size());
			std::cout << fileName << std::endl;
			long pos = fileName.find('_', 0);
			int firstIndex = stoi(fileName.substr(0, long(pos)));

			int pos1 = fileName.find('_', long(pos) + 1);
			int secondIndex = stoi(fileName.substr(long(pos) + 1, pos1));

			int betweenIndex = stoi(fileName.substr(long(pos1) + 1, fileName.size()));

			//�ж϶��ٷ�����
			if (secondIndex + 1 > pointCloudNum);
			{
				pointCloudNum = secondIndex + 1;
			}

			//��ȡ��������
			std::ifstream fin1(filePath);
			std::string line1;
			Eigen::Matrix<double, Eigen::Dynamic, 3> onePointCloud;
			std::vector<Eigen::Vector3d> pointCloud1_vec;
			while (std::getline(fin1, line1))
			{
				std::stringstream ss(line1);
				double point_X;
				double point_Y;
				double point_Z;
				ss >> point_X >> point_Y >> point_Z;
				Eigen::Vector3d point(point_X, point_Y, point_Z);

				pointCloud1_vec.push_back(point);
			}
			fin1.close();

			onePointCloud.conservativeResize(pointCloud1_vec.size() + 1, Eigen::NoChange);
			Eigen::Vector3d indexVec(firstIndex, secondIndex, betweenIndex);
			onePointCloud.row(0) = indexVec;
			for (int k = 1; k < pointCloud1_vec.size() + 1; k++)
			{
				onePointCloud.row(k) = pointCloud1_vec[k - 1];
			}

			if (firstIndex == betweenIndex)
			{
				PointCloudMat[i / 2].first = onePointCloud;
			}
			else
			{
				PointCloudMat[i / 2].second = onePointCloud;
			}



			//std::cout << fileName << " " << firstIndex << " " << secondIndex<<" "<< betweenIndex << std::endl;;
		}

		/*for (int i=0;i< PointCloudMat.size();i++)
		{
			Eigen::Vector3d p1= PointCloudMat[i].first.row(0);
			Eigen::Vector3d p2= PointCloudMat[i].second.row(0);
			std::cout << "-------------" << std::endl;
			std::cout << p1 << std::endl << p2 << std::endl;


			Eigen::MatrixXd mat1 = (PointCloudMat[i].first).block(1, 0, (PointCloudMat[i].first).rows() - 1, (PointCloudMat[i].first).cols());
			Eigen::MatrixXd mat2 = (PointCloudMat[i].second).block(1, 0, (PointCloudMat[i].second).rows() - 1, (PointCloudMat[i].second).cols());

			std::cout << "mat1: " << std::endl;
			std::cout << mat1 << std::endl;
			std::cout << "mat2: " << std::endl;
			std::cout << mat2 << std::endl;
		}*/



		//����ÿ�����ƶ�Ӧ��RT
		Eigen::Vector3d* rotationVec = new Eigen::Vector3d[pointCloudNum];
		Eigen::Vector3d* translation = new Eigen::Vector3d[pointCloudNum];
		//��ת������ƽ��������ʼ��
		for (int i = 0; i < pointCloudNum; i++)
		{
			rotationVec[i] = Eigen::Vector3d(0, 0, 0);
			translation[i] = Eigen::Vector3d(0, 0, 0);
		}

		//��¼����ͶӰ���(dx2+dy2+dz2)
		std::vector<double> err_pre;
		std::vector<double> err_cur;



		//�Ż�֮ǰ
		//�������֮��ƽ�����
		Eigen::Vector3d err0(0, 0, 0);
		for (int i = 0; i < PointCloudMat.size(); i++)
		{
			//ȡ����Ӧ�����±�
			int index1 = -1, index2 = -1;
			index1 = (int)(PointCloudMat[i].first).row(0)(2);
			index2 = (int)(PointCloudMat[i].second).row(0)(2);

			//ȡ����Ӧ����
			Eigen::MatrixXd pointCloud_pre, pointCloud_cur;
			pointCloud_pre = (PointCloudMat[i].first).block(1, 0, (PointCloudMat[i].first).rows() - 1, (PointCloudMat[i].first).cols());
			pointCloud_cur = (PointCloudMat[i].second).block(1, 0, (PointCloudMat[i].second).rows() - 1, (PointCloudMat[i].second).cols());

			Eigen::MatrixXd pointMat_pre1 = pointCloud_pre.transpose();
			Eigen::MatrixXd pointMat_cur1 = pointCloud_cur.transpose();

			//ȡ����ǰRT
			Eigen::Vector3d RVec_pre = rotationVec[index1];
			Eigen::Matrix3d R_Mat_pre;
			R_Mat_pre = Eigen::AngleAxisd(RVec_pre.norm(), RVec_pre.normalized());

			Eigen::Vector3d RVec_cur = rotationVec[index2];
			Eigen::Matrix3d R_Mat_cur;
			R_Mat_cur = Eigen::AngleAxisd(RVec_cur.norm(), RVec_cur.normalized());

			Eigen::Vector3d T_Vec_pre = translation[index1];
			Eigen::Vector3d T_Vec_cur = translation[index2];

			Eigen::MatrixXd errMat = R_Mat_cur * pointMat_cur1 - (R_Mat_pre * pointMat_pre1);
			for (int m = 0; m < errMat.cols(); m++)
			{
				errMat(0, m) = errMat(0, m) + T_Vec_cur(0) - T_Vec_pre(0);
				errMat(1, m) = errMat(1, m) + T_Vec_cur(1) - T_Vec_pre(1);
				errMat(2, m) = errMat(2, m) + T_Vec_cur(2) - T_Vec_pre(2);
			}


			Eigen::Vector3d errColumAbsSum(0, 0, 0);

			double err_every = 0;
			for (int m = 0; m < errMat.cols(); m++)
			{
				errColumAbsSum(0) += abs(errMat(0, m));
				errColumAbsSum(1) += abs(errMat(1, m));
				errColumAbsSum(2) += abs(errMat(2, m));


				err_every += sqrt(errMat(0, m) * errMat(0, m) + errMat(1, m) * errMat(1, m) + errMat(2, m) * errMat(2, m));
			}
			err_every = err_every / errMat.cols();
			err_pre.push_back(err_every);

			errColumAbsSum = errColumAbsSum / errMat.cols();

			err0 = err0 + errColumAbsSum;
			std::cout << "--------------------------" << std::endl;
			std::cout << "---------" << "pointCloud " << i + 1 << " to " << "pointCloud " << i << "---------" << std::endl;
			std::cout << "preErr_X: " << errColumAbsSum(0) << " preErr_Y " << errColumAbsSum(1) << " preErr_Z " << errColumAbsSum(2) << std::endl;


		}
		err0 = err0 / (pointCloudNum - 1);

		std::cout << "----------------���е����Ż�ǰx,y,z��Ӧ���ľ���ֵȡƽ��---------------" << std::endl;
		std::cout << err0 << std::endl;


		ceres::Problem problem0;
		for (int i = 0; i < PointCloudMat.size(); i++)
		{
			//ȡ����Ӧ�����±�
			int index1 = -1, index2 = -1;
			index1 = (int)(PointCloudMat[i].first).row(0)(2);
			index2 = (int)(PointCloudMat[i].second).row(0)(2);

			//ȡ����Ӧ����
			Eigen::MatrixXd pointCloud_pre, pointCloud_cur;
			pointCloud_pre = (PointCloudMat[i].first).block(1, 0, (PointCloudMat[i].first).rows() - 1, (PointCloudMat[i].first).cols());
			pointCloud_cur = (PointCloudMat[i].second).block(1, 0, (PointCloudMat[i].second).rows() - 1, (PointCloudMat[i].second).cols());

			//����ÿһ����
			for (int k = 0; k < pointCloud_pre.rows(); k++)
			{
				Eigen::Vector3d point_pre = pointCloud_pre.row(k);
				Eigen::Vector3d point_cur = pointCloud_cur.row(k);

				//ȡ�����������Ӧ��R��T
				problem0.AddResidualBlock(
					new ceres::AutoDiffCostFunction<optimatePointCloutRT, 3, 3, 3, 3, 3>(new optimatePointCloutRT(point_pre, point_cur)),
					nullptr,
					rotationVec[index1].data(),
					translation[index1].data(),
					rotationVec[index2].data(),
					translation[index2].data()
				);
			}

		}

		clock_t startTime, endTime;
		startTime = clock();

		//���õ�һ֡���Ƶ�RT����
		problem0.SetParameterBlockConstant(rotationVec[0].data());
		problem0.SetParameterBlockConstant(translation[0].data());
		// �����Ż�ѡ��
		ceres::Solver::Options options0;
		//options0.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
		options0.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
		options0.minimizer_progress_to_stdout = false;
		options0.max_num_iterations = 300;
		options0.parameter_tolerance = 1e-15;

		// ִ���Ż�
		ceres::Solver::Summary summary0;
		ceres::Solve(options0, &problem0, &summary0);

		endTime = clock();

		//�����ת����
		for (int i = 0; i < pointCloudNum; i++)
		{
			//ȡ��RT
			Eigen::Vector3d R_Vec = rotationVec[i];
			Eigen::Vector3d T_Vec = translation[i];


			Eigen::Matrix3d R_Mat;
			R_Mat = Eigen::AngleAxisd(R_Vec.norm(), R_Vec.normalized());
			std::cout << "-------------------------------" << std::endl;
			std::cout << "index: " << i << std::endl;
			std::cout << "R_Mat: " << std::endl;
			std::cout << R_Mat << std::endl;
			std::cout << "T_Vec : " << std::endl;
			std::cout << T_Vec << std::endl;
			Eigen::Matrix4d transform_1 = Eigen::Matrix4d::Identity();
			transform_1.block(0, 0, 3, 3) = R_Mat;
			std::cout << transform_1 << std::endl;
			transform_1.block(0, 3, 3, 1) = T_Vec;
			std::cout << transform_1 << std::endl;
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
			std::string PLY = folder + std::to_string(i) + ".ply";
			pcl::io::loadPLYFile<pcl::PointXYZ>(PLY, *source_cloud);
			pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);
			pcl::io::savePLYFile(PLY, *transformed_cloud);
		}


		Eigen::Vector3d err1(0, 0, 0);//���ж�Ӧ���Ƶ�x,y,z����ֵ����ƽ��
		//�������֮��ƽ�����
		for (int i = 0; i < PointCloudMat.size(); i++)
		{
			//ȡ����Ӧ�����±�
			int index1 = -1, index2 = -1;
			index1 = (int)(PointCloudMat[i].first).row(0)(2);
			index2 = (int)(PointCloudMat[i].second).row(0)(2);

			//ȡ����Ӧ����
			Eigen::MatrixXd pointCloud_pre, pointCloud_cur;
			pointCloud_pre = (PointCloudMat[i].first).block(1, 0, (PointCloudMat[i].first).rows() - 1, (PointCloudMat[i].first).cols());
			pointCloud_cur = (PointCloudMat[i].second).block(1, 0, (PointCloudMat[i].second).rows() - 1, (PointCloudMat[i].second).cols());

			Eigen::MatrixXd pointMat_pre1 = pointCloud_pre.transpose();
			Eigen::MatrixXd pointMat_cur1 = pointCloud_cur.transpose();

			//ȡ����ǰRT
			Eigen::Vector3d RVec_pre = rotationVec[index1];
			Eigen::Matrix3d R_Mat_pre;
			R_Mat_pre = Eigen::AngleAxisd(RVec_pre.norm(), RVec_pre.normalized());

			Eigen::Vector3d RVec_cur = rotationVec[index2];
			Eigen::Matrix3d R_Mat_cur;
			R_Mat_cur = Eigen::AngleAxisd(RVec_cur.norm(), RVec_cur.normalized());

			Eigen::Vector3d T_Vec_pre = translation[index1];
			Eigen::Vector3d T_Vec_cur = translation[index2];

			Eigen::MatrixXd errMat = R_Mat_cur * pointMat_cur1 - (R_Mat_pre * pointMat_pre1);
			for (int m = 0; m < errMat.cols(); m++)
			{
				errMat(0, m) = errMat(0, m) + T_Vec_cur(0) - T_Vec_pre(0);
				errMat(1, m) = errMat(1, m) + T_Vec_cur(1) - T_Vec_pre(1);
				errMat(2, m) = errMat(2, m) + T_Vec_cur(2) - T_Vec_pre(2);
			}

			Eigen::Vector3d errColumAbsSum(0, 0, 0);//x,y,z�ĸ���������ֵ֮�͵�ƽ��
			double err_every = 0;//err_every���ڱ�ʾ��Ӧ���ƾ���ƽ����
			for (int m = 0; m < errMat.cols(); m++)
			{
				errColumAbsSum(0) += abs(errMat(0, m));
				errColumAbsSum(1) += abs(errMat(1, m));
				errColumAbsSum(2) += abs(errMat(2, m));

				err_every += sqrt(errMat(0, m) * errMat(0, m) + errMat(1, m) * errMat(1, m) + errMat(2, m) * errMat(2, m));
			}
			err_every = err_every / errMat.cols();
			err_cur.push_back(err_every);

			errColumAbsSum = errColumAbsSum / errMat.cols();
			err1 = err1 + errColumAbsSum;

			std::cout << "--------------------------" << std::endl;
			std::cout << "---------" << "pointCloud " << i + 1 << " to " << "pointCloud " << i << "---------" << std::endl;
			std::cout << "err_X: " << errColumAbsSum(0) << " err_Y " << errColumAbsSum(1) << " err_Z " << errColumAbsSum(2) << std::endl;

		}

		err1 = err1 / (pointCloudNum - 1);
		std::cout << "----------------���е����Ż���x,y,z��Ӧ���ľ���ֵȡƽ��---------------" << std::endl;
		std::cout << err1 << std::endl;


		std::cout << "****************�Ż�ǰ�����Ա�**********************" << std::endl;
		std::cout << "�Ż���sqrt(dx2+dy2+dz2)��ȥ�Ż�ǰsqrt(dx2+dy2+dz2)��ÿ��ͼ��" << std::endl;
		for (int i = 0; i < err_pre.size(); i++)
		{
			std::cout << "index: " << i << " " << i + 1 << std::endl;
			std::cout << err_cur[i] - err_pre[i] << std::endl;
		}

		std::cout << "x,y,z����ͼ���ƽ���仯��(�Ż����ȥ�Ż�ǰ)��" << std::endl;
		std::cout << err1 - err0 << std::endl;

		std::cout << "spendTime: " << endTime - startTime << " ms" << std::endl;
	}
	clock_t end = clock();
	std::cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << std::endl;

	CalAfterErr();
}
