//�����ں˵�ͷ�ļ�
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//���������ڵ�ͷ�ļ�
#include <CGAL/Delaunay_triangulation_3.h>
//��Ȼ�����ֵ������ͷ�ļ�
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
//��ʾ���ڼ��μ�����ںˣ�kernel����
//typedef CGAL::Exact_predicates_inexact_constructions_kernelָ����һ���ṩ��ȷν�ʣ����ԣ��Ͳ���ȷ���죨���㣩���ںˡ�
//����CGAL�г��õ��ں�֮һ��
typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;


//ʹ���ں�K��ʾ��ά�ռ��е�һ����
typedef K::Point_3                                             Point3;
//ʹ���ں�K��ʾ��ά�ռ��е�һ������
typedef K::Vector_3                                            Vector3;
//ʹ���ں�K��ʾ��ά�ռ��е�һ����
typedef K::Sphere_3                                            Sphere_3;
//ʹ���ں�K����άDelaunay���ǻ����ݽṹ��
typedef CGAL::Delaunay_triangulation_3<K, CGAL::Fast_location> Dh;

//��������һЩ���ڷ������Dh���ݽṹ��typedef
//��άDelaunay���ǻ���������
typedef Dh::Facet                                              Facet;
//����������
typedef Dh::Vertex_handle                                      Vertex_handle;
//�����嵥Ԫ�������
typedef Dh::Cell_handle                                        Cell_handle;
//�������޶���ĵ���������
typedef Dh::Finite_vertices_iterator                           Finite_vertices_iterator;
//��������ĵ���������
typedef Dh::Vertex_iterator                                    Vertex_iterator;
//�������ѭ������������
typedef Dh::Facet_circulator                                   Facet_circulator;
//������Ԫ�ĵ���������
typedef Dh::Cell_iterator                                      Cell_iterator;

//�������ͣ�ʹ���ں�K�е�FT���ͣ��������ͣ�
typedef K::FT                                                  Coord_type;
//������ά�ռ��е����Բ�ĵĺ�������
typedef K::Construct_circumcenter_3                            Construct_circumcenter_3;




void ScatteredInterpolant(Dh& triangulation, Point3& pp, std::vector< std::pair< Vertex_handle, NT> >& coor_sibson, NT& norm_coeff_sibson, double& result, std::map<Point3, double>& value_f) {
	//��sibson��ֵ���������빹������ά��������triangulation����ii����ֵ�㡣
	//���������õ�ÿ��������Ȩ��coor_sibson���͹�һ��ϵ��norm_coeff_sibson
	sibson_natural_neighbor_coordinates_3(triangulation, pp,
		std::back_inserter(coor_sibson),
		norm_coeff_sibson);

	//std::cout << "ÿ��������NT��" << std::endl;
	for (int i = 0; i < coor_sibson.size(); ++i) {
		//std::cout << coor_sibson[i].second << std::endl;
		result += coor_sibson[i].second * value_f[coor_sibson[i].first->point()];
	}

	//std::cout << "��һ��ϵ��norm_coeff_sibsonΪ��" << std::endl;
	//std::cout << norm_coeff_sibson << std::endl;
	//std::cout << "��ֵ���Ϊ��" << std::endl;
}

int main()
{
	//��ȡ������value
	clock_t start1, end1;
	start1 = clock();
	std::fstream inFile_value;
	inFile_value.open("E:\\XTOP\\XTOPwork\\scatteredInterpolant\\��������\\������.txt");
	if (!inFile_value) {
		std::cout << "Unable to open file";
		exit(1); // terminate with error
	}

	//��ȡ����������
	clock_t start2, end2;
	start2 = clock();
	std::fstream iFile("E:\\XTOP\\XTOPwork\\scatteredInterpolant\\��������\\����������.txt", std::ios::in);

	Point3 p;
	double value;


	Dh triangulation;
	int m = 0;
	while (iFile >> p) {
		inFile_value >> value;
		//�����뷨������ά��������
		p.value = value;
		triangulation.insert(p);
	}
	end2 = clock();
	std::cout << "��������ʱ�䣺" << double(end2 - start2) / CLOCKS_PER_SEC << "s" << std::endl;


	//����ֵ��
	clock_t start3, end3;
	start3 = clock();
	//�Ѳ�ֵ�㶼�浽pp���������
	std::vector<Point3> pp;
	std::fstream iFile_query("E:\\XTOP\\XTOPwork\\scatteredInterpolant\\��������\\һ�������ֵ������.txt", std::ios::in);
	Point3 p1;
	while (iFile_query >> p1) {
		pp.push_back(p1);
	}
	end3 = clock();
	std::cout << "����ֵ��ʱ�䣺" << double(end3 - start3) / CLOCKS_PER_SEC << "s" << std::endl;

	std::ofstream query_value("��ֵ��value2.txt", std::ios::out);
	clock_t start4, end4;
	start4 = clock();


	omp_set_num_threads(4);
#pragma omp parallel for
	//��ֵ
	for (int ii = 0; ii < pp.size(); ++ii)
	{
		//�Ľ�����㷨
		Fast_ScatteredInterpolant(triangulation, pp[ii]);
		//ScatteredInterpolant(triangulation, pp[ii], coor_sibson, norm_coeff_sibson, result, value_f);
	}
	end4 = clock();

	//д�ļ�
	for (int i = 0; i < pp.size(); ++i){
		query_value << pp[i].value << std::endl;
	}
	query_value.close();


	std::cout << "��ֵʱ�� = " << double(end4 - start4) / CLOCKS_PER_SEC << "s" << std::endl;  //���ʱ�䣨��λ����
	system("pause");
	return EXIT_SUCCESS;
}