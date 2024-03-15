//定义内核的头文件
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//构建德劳内的头文件
#include <CGAL/Delaunay_triangulation_3.h>
//自然邻域插值函数的头文件
#include <CGAL/natural_neighbor_coordinates_3.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <utility>
#include <vector>
#include <map>
#include <math.h>
#include <iomanip>
#include <time.h>
#include <omp.h>


typedef double NT; //Number Type
//表示用于几何计算的内核（kernel）。
//typedef CGAL::Exact_predicates_inexact_constructions_kernel指定了一个提供精确谓词（测试）和不精确构造（计算）的内核。
//这是CGAL中常用的内核之一。
typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;


//使用内核K表示三维空间中的一个点
typedef K::Point_3                                             Point3;
//使用内核K表示三维空间中的一个向量
typedef K::Vector_3                                            Vector3;
//使用内核K表示三维空间中的一个球。
typedef K::Sphere_3                                            Sphere_3;
//使用内核K的三维Delaunay三角化数据结构。
typedef CGAL::Delaunay_triangulation_3<K, CGAL::Fast_location> Dh;

//接下来是一些用于方便访问Dh数据结构的typedef
//三维Delaunay三角化的面类型
typedef Dh::Facet                                              Facet;
//顶点句柄类型
typedef Dh::Vertex_handle                                      Vertex_handle;
//四面体单元句柄类型
typedef Dh::Cell_handle                                        Cell_handle;
//遍历有限顶点的迭代器类型
typedef Dh::Finite_vertices_iterator                           Finite_vertices_iterator;
//遍历顶点的迭代器类型
typedef Dh::Vertex_iterator                                    Vertex_iterator;
//遍历面的循环迭代器类型
typedef Dh::Facet_circulator                                   Facet_circulator;
//遍历单元的迭代器类型
typedef Dh::Cell_iterator                                      Cell_iterator;

//坐标类型，使用内核K中的FT类型（浮点类型）
typedef K::FT                                                  Coord_type;
//构造三维空间中的外接圆心的函数类型
typedef K::Construct_circumcenter_3                            Construct_circumcenter_3;




void ScatteredInterpolant(Dh& triangulation, Point3& pp, std::vector< std::pair< Vertex_handle, NT> >& coor_sibson, NT& norm_coeff_sibson, double& result, std::map<Point3, double>& value_f) {
	//用sibson插值函数，输入构建的三维德劳内网triangulation，第ii个插值点。
	//函数会计算得到每个邻域点的权重coor_sibson，和归一化系数norm_coeff_sibson
	sibson_natural_neighbor_coordinates_3(triangulation, pp,
		std::back_inserter(coor_sibson),
		norm_coeff_sibson);

	//std::cout << "每个邻域点的NT：" << std::endl;
	for (int i = 0; i < coor_sibson.size(); ++i) {
		//std::cout << coor_sibson[i].second << std::endl;
		result += coor_sibson[i].second * value_f[coor_sibson[i].first->point()];
	}

	//std::cout << "归一化系数norm_coeff_sibson为：" << std::endl;
	//std::cout << norm_coeff_sibson << std::endl;
	//std::cout << "插值结果为：" << std::endl;
}

int main()
{
	//读取样本点value
	clock_t start1, end1;
	start1 = clock();
	std::fstream inFile_value;
	inFile_value.open("E:\\XTOP\\XTOPwork\\scatteredInterpolant\\测试数据\\样本点.txt");
	if (!inFile_value) {
		std::cout << "Unable to open file";
		exit(1); // terminate with error
	}

	//读取样本点坐标
	clock_t start2, end2;
	start2 = clock();
	std::fstream iFile("E:\\XTOP\\XTOPwork\\scatteredInterpolant\\测试数据\\样本点坐标.txt", std::ios::in);

	Point3 p;
	double value;


	Dh triangulation;
	int m = 0;
	while (iFile >> p) {
		inFile_value >> value;
		//逐点插入法建立三维德劳内网
		p.value = value;
		triangulation.insert(p);
	}
	end2 = clock();
	std::cout << "读样本点时间：" << double(end2 - start2) / CLOCKS_PER_SEC << "s" << std::endl;


	//读插值点
	clock_t start3, end3;
	start3 = clock();
	//把插值点都存到pp这个容器中
	std::vector<Point3> pp;
	std::fstream iFile_query("E:\\XTOP\\XTOPwork\\scatteredInterpolant\\测试数据\\一百万个插值点坐标.txt", std::ios::in);
	Point3 p1;
	while (iFile_query >> p1) {
		pp.push_back(p1);
	}
	end3 = clock();
	std::cout << "读插值点时间：" << double(end3 - start3) / CLOCKS_PER_SEC << "s" << std::endl;

	std::ofstream query_value("插值点value2.txt", std::ios::out);
	clock_t start4, end4;
	start4 = clock();


	omp_set_num_threads(4);
#pragma omp parallel for
	//插值
	for (int ii = 0; ii < pp.size(); ++ii)
	{
		//改进后的算法
		Fast_ScatteredInterpolant(triangulation, pp[ii]);
		//ScatteredInterpolant(triangulation, pp[ii], coor_sibson, norm_coeff_sibson, result, value_f);
	}
	end4 = clock();

	//写文件
	for (int i = 0; i < pp.size(); ++i){
		query_value << pp[i].value << std::endl;
	}
	query_value.close();


	std::cout << "插值时间 = " << double(end4 - start4) / CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
	system("pause");
	return EXIT_SUCCESS;
}