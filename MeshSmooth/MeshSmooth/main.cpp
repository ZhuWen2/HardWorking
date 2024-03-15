// MeshSmooth.cpp : Defines the entry point for the console application.


#include "stdafx.h"
#include "MeshSmooth.h"
#include <iostream>
#include <time.h>

//�趨�ֳ������ֻ���������model��
//���û������ʸߵ͵���ֵ���������������������Ƭ�н���һЩ����min_angle�ģ���˵�������ʽϸߵĲ��֡�
//��������Ǻ�ϸ�壬�뾶radius��Ҫ��СһЩ��
//smooth_area����ͬ����1Ϊ�����˳��2��3��4��������ʴӸߵ��͡�0Ϊ���������������ʲ�ͬ�ط�����Ƭ����idx�������浽������ͬ������
//��������ֲܴڵĻ������Ƚ���������˹ȥ�룬��֮���˫���˲�һ���ȽϺõĳ�ֵ��
//��η���������һ����Ƭ��Χ�����棬���������������Ƭ�ķ��߼нǣ�ͳ�ƴ���min_angle���������������һ����������˵����������������ʱȽϸߵĲ��֣�
//�������ֵ�ĳ�������С�

int model = 0;
double min_angle = 7;
int radius = 1;
int smooth_area = 1;
int max_sharp_num = 0;
int sharp_num = 0;
//����������Ƭ������������
std::set<const int> face_arris1;
std::set<const int> face_arris2;
std::set<const int> face_arris3;
std::map<const int, int> sharp_face;
std::priority_queue<std::pair<double, std::pair<myMesh::FaceHandle, myMesh::FaceHandle>>, std::vector<std::pair<double, std::pair<myMesh::FaceHandle, myMesh::FaceHandle>>>, std::less<std::pair<double, std::pair<myMesh::FaceHandle, myMesh::FaceHandle>>> > pq;
std::map<const int, myMesh::Normal> sharp_face_normal;
std::set<const int> ffww;

int main(/*int argc, char* argv[]*/)
{
	myMesh mesh;
	std::string path = "C:\\Users\\123456\\Desktop\\���Թ���\\����10MLS.stl";
	if (!OpenMesh::IO::read_mesh(mesh, path))
	{
		std::cerr << "Read mesh failed!" << std::endl;
		return -1;
	}
	mesh.request_face_normals();
	mesh.update_face_normals();
	mesh.request_vertex_normals();
	mesh.update_vertex_normals();
	
	//ɾ��Ԫ�ص�׼������
	mesh.request_face_status();
	mesh.request_edge_status();
	mesh.request_vertex_status();
	

	clock_t start = clock();
	GELSmooth gel;
	//��������΢��˳һ�£��ﵽ�ȽϹ⻬��Ч����Ȼ�󻮷ֳ����ʸߺ����ʵ͵�λ��

	//----------------------------��һ���˳-----------------------------
	//gel.LaplaceSmooth(mesh, 5);
	//gel.denoise(mesh, 5);//����������
	if (model == 1) {
		radius = 2;
		smooth_area = 1;
		gel.denoise(mesh, 10);//����������
		gel.optimize_mesh(mesh);
	}

	else if (model == 2){
		//------------------------------��������------------------------------
		radius = 1;
		smooth_area = 0;
		gel.MLSSmooth(mesh, 1);
		std::cout << face_arris1.size() << std::endl;
		std::cout << face_arris2.size() << std::endl;
		//-----------------------------ɾ����Ƭ----------------------------------

		for (auto i = face_arris2.cbegin(); i != face_arris2.cend(); i++) {
		myMesh::FaceHandle fhandle(*i);

		mesh.delete_face(fhandle, false);
		}
		mesh.garbage_collection();
		//---------------------------�ڶ����˳--------------------------------
		radius = 1;
		smooth_area = 2;
		//gel.LaplaceSmooth(mesh, 10);
		//gel.MLSSmooth(mesh, 1);


		//-------------------------------�������˳-------------------------------
		radius = 2;
		smooth_area = 3;
		//gel.MLSSmooth(mesh, 5);

		//-------------------------------����������Թ�˳һ��ʹ��ͬ����Ľ��紦��ƽ��--------------------------------
		radius = 1;
		smooth_area = 1;
		//gel.denoise(mesh, 2);//����������
		//gel.LaplaceSmooth(mesh, 20);
		gel.optimize_mesh(mesh);
	}
	else if (model == 3){
		//------------------------------��������------------------------------
		radius = 1;
		smooth_area = 0;
		gel.denoise(mesh, 1);
		std::cout << face_arris1.size() << std::endl;
		std::cout << face_arris2.size() << std::endl;
		std::cout << face_arris3.size() << std::endl;

		//---------------------------�ڶ����˳--------------------------------
		radius = 1;
		smooth_area = 2;
		gel.denoise(mesh, 10);
		//-----------------------------ɾ����Ƭ----------------------------------

		/*for (auto i = face_arris2.cbegin(); i != face_arris2.cend(); i++) {
		myMesh::FaceHandle fhandle(*i);

		mesh.delete_face(fhandle, false);
		}
		mesh.garbage_collection();*/

		//-------------------------------�������˳-------------------------------
		radius = 1;
		smooth_area = 3;
		gel.denoise(mesh, 10);
		//gel.LaplaceSmooth(mesh, 10);

		//-------------------------------���ı��˳--------------------------------
		radius = 2;
		smooth_area = 4;
		gel.denoise(mesh, 25);
		//gel.LaplaceSmooth(mesh, 10);

		//-------------------------------����������Թ�˳һ��ʹ��ͬ����Ľ��紦��ƽ��--------------------------------
		radius = 1;
		smooth_area = 1;
		gel.denoise(mesh, 5);//����������
		//gel.LaplaceSmooth(mesh, 5);
		gel.optimize_mesh(mesh);
	}
	
	clock_t end = clock();
	std::cout << "time = " << double(end-start) / CLOCKS_PER_SEC << "s" << std::endl;  //���ʱ�䣨��λ����

	/*std::string outPath = "C:\\Users\\123456\\Desktop\\���Թ���\\����9h.stl";
	if (!OpenMesh::IO::write_mesh(mesh, outPath))
	{
		std::cerr << "Write mesh failed!" << std::endl;
		return -1;
	}*/





//-------------------------��-----------------------------
	int count_s = 0;
	std::vector<std::vector<const int> > sharp_face_neiidx;
	radius = 1;
	clock_t start1 = clock();
	for (int i = 0; i < 2; ++i) {
		gel.Sharp(mesh, 5, count_s, sharp_face_neiidx);
		//gel.LaplaceSmooth(mesh, 3);

	}
	gel.Sharp(mesh, 5, count_s, sharp_face_neiidx);
	//gel.LaplaceSmooth(mesh, 1);
	clock_t end1 = clock();
	std::cout << "time = " << double(end1 - start1) / CLOCKS_PER_SEC << "s" << std::endl;  //���ʱ�䣨��λ����
	//gel.LaplaceSmooth(mesh, 1);
	//----------------------------˫���˲�----------------------------------
	//model = 2;
	//min_angle = 35;
	//radius = 2;
	//smooth_area = 0;
	//gel.MLSSmooth(mesh, 1);
	//std::cout << face_arris1.size() << std::endl;
	//std::cout << face_arris2.size() << std::endl;
	//
	//radius = 1;
	//smooth_area = 2;
	////gel.LaplaceSmooth(mesh, 10);
	//gel.denoise(mesh, 5);


	//radius = 2;
	//smooth_area = 3;
	////gel.denoise(mesh, 1);

	//gel.optimize_mesh(mesh);


	//----------------------------MLS----------------------------------
	//min_angle = 30;
	//model = 2;
	//radius = 1;
	//smooth_area = 0;
	//gel.MLSSmooth(mesh, 1);
	//std::cout << face_arris1.size() << std::endl;
	//std::cout << face_arris2.size() << std::endl;


	//radius = 2;
	//smooth_area = 2;
	////gel.LaplaceSmooth(mesh, 10);
	//gel.MLSSmooth(mesh, 3);


	//radius = 2;
	//smooth_area = 3;
	//gel.MLSSmooth(mesh, 3);
	//gel.LaplaceSmooth(mesh, 10);




	//gel.Sharp(mesh, 15, count_s, sharp_face_neiidx);
	//gel.LaplaceSmooth(mesh, 5);
	//gel.Sharp(mesh, 20, count_s, sharp_face_neiidx);
	/*gel.LaplaceSmooth(mesh, 5);
	gel.Sharp(mesh, 10, count_s, sharp_face_neiidx);*/
	//std::cout << count_s << std::endl;
	//ע����̻߳���vector��pushback��������
	std::cout << sharp_face_neiidx.size() << std::endl;
	std::cout << sharp_face.size() << std::endl;
	std::cout << pq.size() << std::endl;
	std::cout << sharp_face_normal.size() << std::endl;
	//std::cout << sharp_face.size() << std::endl;
	/*for (auto i = sharp_face.cbegin(); i != sharp_face.cend(); i++) {
		myMesh::FaceHandle fhandle(myMesh::FaceHandle((*i).first));
	
	mesh.delete_face(fhandle, false);
	}
	mesh.garbage_collection();*/
	/*for (auto i = ffww.begin(); i != ffww.end(); i++) {
	myMesh::FaceHandle fhandle(*i);

	mesh.delete_face(fhandle, false);
	}
	mesh.garbage_collection();*/
	//radius = 2;
	//smooth_area = 1;
	//gel.denoise(mesh, 5);//����������
	//gel.optimize_mesh(mesh);
	std::string outPath1 = "C:\\Users\\123456\\Desktop\\���Թ���\\11111111111111.stl";
	if (!OpenMesh::IO::write_mesh(mesh, outPath1))
	{
		std::cerr << "Write mesh failed!" << std::endl;
		return -1;
	}
	
	return 0;
}

