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


	////��¼�������Լ���value��map���Ա�֮��ֱ���ҵ�ĳ���������Ӧ��value
	std::map<Point3, double> value_f;


	//��ȡ������value
	clock_t start1, end1;
	start1 = clock();
	std::fstream inFile_value;

	inFile_value.open("E:\\XTOP\\XTOPwork\\CGAL-NNI\\��������\\������.txt");
	if (!inFile_value) {
		std::cout << "Unable to open file";
		exit(1); // terminate with error
	}
	double v = 0;
	//��������value�ȴ浽va��
	std::vector<double> va;
	while (inFile_value >> v) {
		va.push_back(v);
	}

	inFile_value.close();
	end1 = clock();
	std::cout << "���������ֵʱ�䣺" << double(end1 - start1) / CLOCKS_PER_SEC << "s" << std::endl;

	//��ȡ����������
	clock_t start2, end2;
	start2 = clock();
	std::fstream iFile("E:\\XTOP\\XTOPwork\\CGAL-NNI\\��������\\����������.txt", std::ios::in);
	Point3 p;

	Dh triangulation;
	int m = 0;
	while (iFile >> p) {
		//�����뷨������ά��������
		triangulation.insert(p);
		//ͬʱ�Ѹ����������value�浽value_f���map��
		value_f.insert(std::pair<Point3, double>(p, va[m]));
		++m;
	}
	end2 = clock();
	std::cout << "��������ʱ�䣺" << double(end2 - start2) / CLOCKS_PER_SEC << "s" << std::endl;


	//����ֵ��
	clock_t start3, end3;
	start3 = clock();
	//�Ѳ�ֵ�㶼�浽pp���������
	std::vector<Point3> pp;
	std::fstream iFile_query("E:\\XTOP\\XTOPwork\\CGAL-NNI\\��������\\��ֵ������.txt", std::ios::in);
	Point3 p1;
	while (iFile_query >> p1) {
		pp.push_back(p1);
	}
	end3 = clock();
	std::cout << "����ֵ��ʱ�䣺" << double(end3 - start3) / CLOCKS_PER_SEC << "s" << std::endl;
	//pp[0] = Point3(-172.019, - 117.506, - 676.191); //inside data/points3 convex hull
	//std::cout << "P1 is inside the convex hull" << std::endl;
	//pp[1] = Point3(0,0,0.2); //on data/points3 convex hull
	//std::cout << "P2 is on a vertex of the triangulation" << std::endl;
	//pp[2] = Point3(0,2,0); //outside data/points3 convex hull
	//std::cout << "P3 is outside the convex hull" << std::endl;

	std::ofstream query_value;
	query_value.open("E:\\XTOP\\XTOPwork\\CGAL-NNI\\��������\\��ֵ��value.txt", std::ios::out);
	
	clock_t start4, end4;
	start4 = clock();
	end4 = clock();
	//��ֵ
	for (int ii = 0; ii < pp.size(); ++ii)
	{

		std::vector< std::pair< Point3, Coord_type> > coords;

		std::vector< std::pair< Vertex_handle, NT> > coor_sibson;
		NT norm_coeff_sibson;


		double result = 0;
		double norm = 0;

		std::vector<double> weight;

		clock_t start0, end0;
		start0 = clock();
		//�Ľ�����㷨
		//Fast_ScatteredInterpolant(triangulation, pp[ii], result, norm, weight, value_f);
		//�Ľ�ǰ���㷨
		ScatteredInterpolant(triangulation, pp[ii], coor_sibson, norm_coeff_sibson, result, value_f);

		end0 = clock();
		end4 += end0 - start0;

		/*std::cout << "Linear combination of natural neighbors with Sibson natural coordinates" << std::endl;
		std::cout << " + correctness (ensured only with an exact number type)" << std::endl;
		std::cout << is_correct_natural_neighborhood(triangulation, pp[ii],
		coor_sibson.begin(),
		coor_sibson.end(),
		norm_coeff_sibson)
		<< std::endl;*/



		//double result = 0;
		////std::cout << "ÿ��������nt��" << std::endl;
		//for (int i = 0; i < coor_sibson.size(); ++i) {
		//	//std::cout << coor_sibson[i].second << std::endl;
		//	result += coor_sibson[i].second * value_f[coor_sibson[i].first->point()];
		//}

		//std::cout << "��һ��ϵ��norm_coeff_sibsonΪ��" << std::endl;
		//std::cout << norm_coeff_sibson << std::endl;
		//std::cout << "��ֵ���Ϊ��" << std::endl;
		query_value << result / norm_coeff_sibson << std::endl;


	}
	query_value.close();
	//end4 = clock();
	

	std::cout << "��ֵʱ�� = " << double(end4 - start4) / CLOCKS_PER_SEC << "s" << std::endl;  //���ʱ�䣨��λ����
	system("pause");
	return EXIT_SUCCESS;
}