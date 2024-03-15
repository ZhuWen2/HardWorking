// MeshSmooth.cpp : Defines the entry point for the console application.


#include "stdafx.h"
#include "MeshSmooth.h"
#include <iostream>
#include <time.h>

//设定分成两部分还是三部分model；
//设置划分曲率高低的阈值，如果邻域面里面两两面片夹角有一些大于min_angle的，就说明在曲率较高的部分。
//如果网格不是很细腻，半径radius就要调小一些。
//smooth_area代表不同区域，1为整体光顺，2、3、4区域的曲率从高到低。0为分区操作，把曲率不同地方的面片索引idx（）保存到三个不同集合中
//网格如果很粗糙的话可以先进行拉普拉斯去噪，给之后的双边滤波一个比较好的初值。
//如何分区：搜索一个面片周围邻域面，计算邻域面与此面片的法线夹角，统计大于min_angle的数量，如果超过一定比例，则说明这个区域属于曲率比较高的部分，
//把它划分到某个集合中。

int model = 0;
double min_angle = 7;
int radius = 1;
int smooth_area = 1;
int max_sharp_num = 0;
int sharp_num = 0;
//用来保存面片的索引的容器
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
	std::string path = "C:\\Users\\123456\\Desktop\\测试工件\\测试10MLS.stl";
	if (!OpenMesh::IO::read_mesh(mesh, path))
	{
		std::cerr << "Read mesh failed!" << std::endl;
		return -1;
	}
	mesh.request_face_normals();
	mesh.update_face_normals();
	mesh.request_vertex_normals();
	mesh.update_vertex_normals();
	
	//删除元素的准备工作
	mesh.request_face_status();
	mesh.request_edge_status();
	mesh.request_vertex_status();
	

	clock_t start = clock();
	GELSmooth gel;
	//先总体稍微光顺一下，达到比较光滑的效果，然后划分成曲率高和曲率低的位置

	//----------------------------第一遍光顺-----------------------------
	//gel.LaplaceSmooth(mesh, 5);
	//gel.denoise(mesh, 5);//最大迭代次数
	if (model == 1) {
		radius = 2;
		smooth_area = 1;
		gel.denoise(mesh, 10);//最大迭代次数
		gel.optimize_mesh(mesh);
	}

	else if (model == 2){
		//------------------------------划分区域------------------------------
		radius = 1;
		smooth_area = 0;
		gel.MLSSmooth(mesh, 1);
		std::cout << face_arris1.size() << std::endl;
		std::cout << face_arris2.size() << std::endl;
		//-----------------------------删除面片----------------------------------

		for (auto i = face_arris2.cbegin(); i != face_arris2.cend(); i++) {
		myMesh::FaceHandle fhandle(*i);

		mesh.delete_face(fhandle, false);
		}
		mesh.garbage_collection();
		//---------------------------第二遍光顺--------------------------------
		radius = 1;
		smooth_area = 2;
		//gel.LaplaceSmooth(mesh, 10);
		//gel.MLSSmooth(mesh, 1);


		//-------------------------------第三遍光顺-------------------------------
		radius = 2;
		smooth_area = 3;
		//gel.MLSSmooth(mesh, 5);

		//-------------------------------最后整体稍稍光顺一遍使不同区域的交界处更平滑--------------------------------
		radius = 1;
		smooth_area = 1;
		//gel.denoise(mesh, 2);//最大迭代次数
		//gel.LaplaceSmooth(mesh, 20);
		gel.optimize_mesh(mesh);
	}
	else if (model == 3){
		//------------------------------划分区域------------------------------
		radius = 1;
		smooth_area = 0;
		gel.denoise(mesh, 1);
		std::cout << face_arris1.size() << std::endl;
		std::cout << face_arris2.size() << std::endl;
		std::cout << face_arris3.size() << std::endl;

		//---------------------------第二遍光顺--------------------------------
		radius = 1;
		smooth_area = 2;
		gel.denoise(mesh, 10);
		//-----------------------------删除面片----------------------------------

		/*for (auto i = face_arris2.cbegin(); i != face_arris2.cend(); i++) {
		myMesh::FaceHandle fhandle(*i);

		mesh.delete_face(fhandle, false);
		}
		mesh.garbage_collection();*/

		//-------------------------------第三遍光顺-------------------------------
		radius = 1;
		smooth_area = 3;
		gel.denoise(mesh, 10);
		//gel.LaplaceSmooth(mesh, 10);

		//-------------------------------第四遍光顺--------------------------------
		radius = 2;
		smooth_area = 4;
		gel.denoise(mesh, 25);
		//gel.LaplaceSmooth(mesh, 10);

		//-------------------------------最后整体稍稍光顺一遍使不同区域的交界处更平滑--------------------------------
		radius = 1;
		smooth_area = 1;
		gel.denoise(mesh, 5);//最大迭代次数
		//gel.LaplaceSmooth(mesh, 5);
		gel.optimize_mesh(mesh);
	}
	
	clock_t end = clock();
	std::cout << "time = " << double(end-start) / CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）

	/*std::string outPath = "C:\\Users\\123456\\Desktop\\测试工件\\测试9h.stl";
	if (!OpenMesh::IO::write_mesh(mesh, outPath))
	{
		std::cerr << "Write mesh failed!" << std::endl;
		return -1;
	}*/





//-------------------------锐化-----------------------------
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
	std::cout << "time = " << double(end1 - start1) / CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
	//gel.LaplaceSmooth(mesh, 1);
	//----------------------------双边滤波----------------------------------
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
	//注意多线程会让vector的pushback操作出错
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
	//gel.denoise(mesh, 5);//最大迭代次数
	//gel.optimize_mesh(mesh);
	std::string outPath1 = "C:\\Users\\123456\\Desktop\\测试工件\\11111111111111.stl";
	if (!OpenMesh::IO::write_mesh(mesh, outPath1))
	{
		std::cerr << "Write mesh failed!" << std::endl;
		return -1;
	}
	
	return 0;
}

