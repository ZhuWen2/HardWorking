#pragma once
#include<iostream>


#include "ceres\ceres.h"
#include "ceres\rotation.h"
#include"fstream"



struct testOpti
{
public:
	Eigen::Vector3d pointVec;
	double dis2;

public:
	testOpti(Eigen::Vector3d p,double d):pointVec(p),dis2(d){}

	template<typename T>
	bool operator()(const T* const tarPointVec,T* residual)const
	{
		residual[0] = (tarPointVec[0] - (T)pointVec[0]) * (tarPointVec[0] - (T)pointVec[0]) +
			(tarPointVec[1] - (T)pointVec[1]) * (tarPointVec[1] - (T)pointVec[1]) +
			(tarPointVec[2] - (T)pointVec[2]) *(tarPointVec[2] - (T)pointVec[2]) - (T)dis2;

		return true;
	}

};


struct optimatePointCloutRT
{

public:
	Eigen::Vector3d point1Vec;
	Eigen::Vector3d point2Vec;

public:
	optimatePointCloutRT(Eigen::Vector3d point1, Eigen::Vector3d point2):point1Vec(point1), point2Vec(point2){}

	template<typename T>
	bool operator()(const T* const rotationVec1, const T* const translation1, const T* const rotationVec2, const T* const translation2, T* residual)const
	{
		//取出各自的旋转向量和平移向量
		Eigen::Matrix<T, 3, 3> rotationMatrix1;
		ceres::AngleAxisToRotationMatrix(rotationVec1, rotationMatrix1.data());

		Eigen::Matrix<T, 3, 3> rotationMatrix2;
		ceres::AngleAxisToRotationMatrix(rotationVec2, rotationMatrix2.data());


		Eigen::Matrix<T, 3, 1> p1Vec((T)point1Vec(0), (T)point1Vec(1), (T)point1Vec(2));
		Eigen::Matrix<T, 3, 1> p2Vec((T)point2Vec(0), (T)point2Vec(1), (T)point2Vec(2));
	
		Eigen::Matrix<T, 3, 1> trans1(translation1[0], translation1[1], translation1[2]);
		Eigen::Matrix<T, 3, 1> trans2(translation2[0], translation2[1], translation2[2]);


		//旋转点
		 // 应用旋转和平移变换
		Eigen::Matrix<T, 3, 1> transformed_point1 = rotationMatrix1 * p1Vec + trans1;
		Eigen::Matrix<T, 3, 1> transformed_point2 = rotationMatrix2 * p2Vec + trans2;

		//计算残差
		residual[0] = transformed_point2[0] - transformed_point1[0];
		residual[1] = transformed_point2[1] - transformed_point1[1];
		residual[2] = transformed_point2[2] - transformed_point1[2];
		//std::cout << "本次迭代后的残差为：" << residual[0] << std::endl;
		/*residual[0] = sqrt((transformed_point2[0] - transformed_point1[0]) * (transformed_point2[0] - transformed_point1[0]) +
			(transformed_point2[1] - transformed_point1[1]) * (transformed_point2[1] - transformed_point1[1]) +
			(transformed_point2[2] - transformed_point1[2]) * (transformed_point2[2] - transformed_point1[2]));
		*/
		//residual = transformed_point2[0] - transformed_point1[0];
		return true;

	}


};



struct optimateTwoDerivative
{
public:
	double XPos1, XPos2, XPos3, XPos4;

public:
	optimateTwoDerivative(double x1, double x2, double x3, double x4) :XPos1(x1), XPos2(x2), XPos3(x3), XPos4(x4){};

	template<typename T>
	bool operator()(const T* const twoPolpara, T* residual)const
	{
		T a1, a2, a3;
		a1 = twoPolpara[0];
		a2 = twoPolpara[1];
		a3 = twoPolpara[2];

		T posY1 = a1 * (T)XPos1 * (T)XPos1 + a2 * (T)XPos1 + a3;
		T posY2 = a1 * (T)XPos2 * (T)XPos2 + a2 * (T)XPos2 + a3;
		T posY3 = a1 * (T)XPos3 * (T)XPos3 + a2 * (T)XPos3 + a3;
		T posY4 = a1 * (T)XPos4 * (T)XPos4 + a2 * (T)XPos4 + a3;

		
		T derivative_v1 = posY1 - posY2;
		T derivative_v2 = posY2 - posY3;
		T derivative_v3 = posY3 - posY4;

		residual[0] = ((derivative_v1 - derivative_v2) - (derivative_v2 - derivative_v3))* (T)1000;

		return true;

	}

};


struct twoPol
{
public:
	double YPos;
	double XPos;

public:
	twoPol(double Y, double X) :YPos(Y), XPos(X) {};

	template<typename T>
	bool operator()(const T* const twoPolpara, T* residual)const
	{
		T a1, a2, a3;
		a1 = twoPolpara[0];
		a2 = twoPolpara[1];
		a3 = twoPolpara[2];

		residual[0] = a1 * XPos * XPos + a2 * XPos + a3 - YPos;

		return true;

	}



};

struct fourPol
{
public:
	double YPos;
	double XPos;
public:
	fourPol(double Y, double X) :YPos(Y), XPos(X) {};

	template<typename T>
	bool operator()(const T* const fourPolpara, T* residual)const
	{
		T a1, a2, a3, a4, a5;
		a1 = fourPolpara[0];
		a2 = fourPolpara[1];
		a3 = fourPolpara[2];
		a4 = fourPolpara[3];
		a5 = fourPolpara[4];

		residual[0] = a1 * (T)XPos * (T)XPos * (T)XPos * (T)XPos + a2 * (T)XPos * (T)XPos * (T)XPos +
			a3 * (T)XPos * (T)XPos + a4 * (T)XPos + a5- (T)YPos;
			

		return true;

	}


};

struct optimateIndexPoint
{
public:
	double YPos;
	double XPos;

public:
	optimateIndexPoint(double Y, double X) :YPos(Y), XPos(X) {};

	template<typename T>
	bool operator()(const T* const fourPolpara, const T* const twoPolpara, T* residual)const
	{
		T a1, a2, a3, a4, a5;
		a1 = fourPolpara[0];
		a2 = fourPolpara[1];
		a3 = fourPolpara[2];
		a4 = fourPolpara[3];
		a5 = fourPolpara[4];

		T b1, b2, b3;
		b1 = twoPolpara[0];
		b2 = twoPolpara[1];
		b3 = twoPolpara[2];

		//拟合的值尽量相等
		residual[0] = a1 * (T)XPos * (T)XPos * (T)XPos * (T)XPos + a2 * (T)XPos * (T)XPos * (T)XPos +
			a3 * (T)XPos * (T)XPos + a4 * (T)XPos + a5 - (b1 * XPos * XPos + b2 * XPos + b3);
		//拟合的导数尽量相等
		residual[1] = (T)4 * a1 * XPos * XPos * XPos + (T)3 * a2 * XPos * XPos + (T)2 * a3 * XPos + a4 -
			((T)2 * b1 * XPos + b2);

		return true;

	}

};


