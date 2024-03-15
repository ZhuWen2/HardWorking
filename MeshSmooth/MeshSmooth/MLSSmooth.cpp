#include "MeshSmooth.h"
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <minmax.h>
#include <algorithm>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/io/ply_io.h>
typedef OpenMesh::VectorT<float, 3> Vector3;




void GELSmooth::MLSSmooth(myMesh &mesh, int max_iter)
{
	//这个函数是一共迭代max_iter次，每次先用GELSmoothFilter进行双边滤波把每个面的法线调整好
	//并保存在filter_normals中。然后调整点的位置，包括10次迭代，每次迭代中利用半边迭代将每个
	//半边的出发点都操作一遍，这样就能保证每个顶点被操作的次数是它相邻面的个数。
	//最后再把mesh中所有顶点遍历一遍，将刚才计算过的每个顶点的位置信息除以其操作的次数
	//就得到了平均的位置信息npos，最后把此顶点调整到npos位置上。这就是此函数的工作。
	double avg_len = 0.0;//边平均长度

	int e_it = 0, e_size = mesh.n_edges();
//#pragma omp parallel for default(shared) reduction(+:avg_len)
	for (e_it = 0; e_it < e_size; ++e_it)
	{
		avg_len += mesh.calc_edge_length(myMesh::EdgeHandle(e_it));//为什么把所有的边长度加一起
	}

	std::vector<myMesh::Normal> filter_normals;
	filter_normals.resize(mesh.n_faces(), myMesh::Normal(0.0));
	for (int iter = 0; iter < max_iter; ++iter)//迭代max_iter次
	{
		int f_it = 0, f_size = mesh.n_faces();//一共有几个面
//#pragma omp parallel for
		for (f_it = 0; f_it < f_size; ++f_it)//循环f_size次
		{
			GELSmoothFilter_MLSpro(mesh, myMesh::FaceHandle(f_it), avg_len / e_size);//计算每个面的双边滤波后的法线。

		}


		mesh.update_face_normals();
	}
}




void GELSmooth::GELSmoothFilter_MLSpro(myMesh & mesh, myMesh::FaceHandle fh, double avg_len)
{

	//此函数是遍历每个面，用邻域顶点MLS拟合曲面并调整位置，实现平滑。



	//创建此面的法向量filter_normal
	myMesh::Normal filter_normal = myMesh::Normal(0.0, 0.0, 0.0);
	//创建此面邻域面的法向量容器face_neighbor
	std::vector<myMesh::FaceHandle> face_neighbor;
	//计算邻域面索引放入face_neighbor中
	getFaceNeighbor(mesh, fh, face_neighbor, true);

	myMesh::Point  pi = mesh.calc_face_centroid(fh);//计算面中心点
	myMesh::Normal ni = mesh.calc_face_normal(fh);//计算面法线

	if (smooth_area == 2){
		if (face_arris1.find(fh.idx()) == face_arris1.end()) {
			return ;
		}
	}
	if (smooth_area == 3) {
		if (face_arris2.find(fh.idx()) == face_arris2.end()) {
			return ;
		}
	}
	if (smooth_area == 4) {
		if (face_arris3.find(fh.idx()) == face_arris3.end()) {
			return ;
		}
	}


	int count1 = 0;
	int count2 = 0;
	//建立一个容器保存邻域面的法向量，等待后面计算它们两两之间的夹角，取夹角最小的做判断。
	std::vector<myMesh::Normal> nei_normal;

	nei_normal.push_back(ni);
	//遍历这些邻面
	std::set<const int> mls_point;
	for (int i = 0; i < face_neighbor.size(); ++i)
	{


		myMesh::FaceHandle f = face_neighbor[i];

		//如果smooth_area == 2，则邻面f不在face_arris2区域时就不参与滤波计算
		if (smooth_area == 2){
		if (face_arris1.find(f.idx()) == face_arris1.end()) {
		continue;
		}

		}
		/*if (smooth_area == 3){
		if (face_arris2.find(f.idx()) == face_arris2.end()) {
		continue;
		}

		}*/
		



		myMesh::Normal nj = mesh.calc_face_normal(f);

		myMesh::Point  pj = mesh.calc_face_centroid(f);

		double length1 = nj.length();
		double length2 = ni.length();
		//std::cout << length2 << "  ";
		double theta = 180 / 3.14159*acos(OpenMesh::dot(nj, ni) / (length1*length2));
		if (smooth_area == 0) {
			nei_normal.push_back(nj);

		}
		else {
			//用循环器遍历这个面的顶点
			for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f); fv_it.is_valid(); ++fv_it)
			{
				mls_point.insert((*fv_it).idx());//把这个面的三个顶点放到mls_point里
			}
		}

	}



	//if (smooth_area == 2 && neibor_plane < 1) return ni;

	//划分索引保存到集合里
	if (smooth_area == 0){
		for (int i = 0; i<nei_normal.size() - 1; ++i) {
			for (int j = i + 1; j<nei_normal.size(); ++j) {
				double length1 = nei_normal[i].length();
				double length2 = nei_normal[j].length();
				double theta = 180 / 3.14159*acos(OpenMesh::dot(nei_normal[j], nei_normal[i]) / (length1*length2));
				if (theta>min_angle) {
					++count1;
				}
				else if (theta > min_angle / 2) {
					++count2;
				}

			}
		}

		int hhhh = nei_normal.size()*(nei_normal.size() - 1) / 2;

		//根据model划分几个区域
		if (model == 2) {
			if (double(count1) / hhhh >= 0.01) {
				face_arris1.insert(fh.idx());
				return ;

			}
			else {
				face_arris2.insert(fh.idx());
				return ;
			}
			nei_normal.clear();
		}
		else if (model == 3){
			if (double(count1) / hhhh >= 0.01) {
				face_arris1.insert(fh.idx());
				return ;

			}
			else if (double(count2) / hhhh >= 0.01){
				face_arris2.insert(fh.idx());
				return ;
			}
			else {
				face_arris3.insert(fh.idx());
				return ;
			}
			nei_normal.clear();
		}

	}
	//进行mls
	else if (smooth_area==3){
		

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nei(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_nei->width = mls_point.size();
		cloud_nei->height = 1;
		cloud_nei->is_dense = false;
		cloud_nei->points.resize(cloud_nei->width * cloud_nei->height);

		auto j = mls_point.begin();
		//将mls_point中的点存到cloud_nei中
		for (size_t i = 0; i < cloud_nei->points.size(); ++i) {
			cloud_nei->points[i].x = mesh.point(myMesh::VertexHandle(*j))[0];
			cloud_nei->points[i].y = mesh.point(myMesh::VertexHandle(*j))[1];
			cloud_nei->points[i].z = mesh.point(myMesh::VertexHandle(*j))[2];
			++j;
		}
		j = mls_point.begin();

		//mls
		// 创建 KD-Tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		// Output has the PointNormal type in order to store the normals calculated by MLS
	
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
		// 定义最小二乘实现的对象mls
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

		mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

		// Set parameters
		mls.setInputCloud(cloud_nei);
		mls.setPolynomialOrder(3); //设置false，可以加速平滑
		mls.setSearchMethod(tree);
		mls.setSearchRadius(7); //单位m，设置用于拟合的K近邻半径
		//mls.setNumberOfThreads(4);
		// Reconstruct
		mls.process(*cloud_out); //输出

		for (size_t i = 0; i < cloud_out->points.size(); ++i) {
			myMesh::Point p;
			p[0] = cloud_out->points[i].x;
			p[1] = cloud_out->points[i].y;
			p[2] = cloud_out->points[i].z;
			mesh.set_point(myMesh::VertexHandle(*j), p);
			++j;
		}
		mls_point.clear();
	}
	else if (smooth_area ==2){


		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nei(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_nei->width = mls_point.size();
		cloud_nei->height = 1;
		cloud_nei->is_dense = false;
		cloud_nei->points.resize(cloud_nei->width * cloud_nei->height);

		auto j = mls_point.begin();
		//将mls_point中的点存到cloud_nei中
		for (size_t i = 0; i < cloud_nei->points.size(); ++i) {
			cloud_nei->points[i].x = mesh.point(myMesh::VertexHandle(*j))[0];
			cloud_nei->points[i].y = mesh.point(myMesh::VertexHandle(*j))[1];
			cloud_nei->points[i].z = mesh.point(myMesh::VertexHandle(*j))[2];
			++j;
		}
		j = mls_point.begin();

		//mls
		// 创建 KD-Tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		// Output has the PointNormal type in order to store the normals calculated by MLS

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
		// 定义最小二乘实现的对象mls
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

		mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

		// Set parameters
		mls.setInputCloud(cloud_nei);
		mls.setPolynomialOrder(3); //设置false，可以加速平滑
		mls.setSearchMethod(tree);
		mls.setSearchRadius(7); //单位m，设置用于拟合的K近邻半径
		//mls.setNumberOfThreads(4);
		// Reconstruct
		mls.process(*cloud_out); //输出

		for (size_t i = 0; i < cloud_out->points.size(); ++i) {
			myMesh::Point p;
			p[0] = cloud_out->points[i].x;
			p[1] = cloud_out->points[i].y;
			p[2] = cloud_out->points[i].z;
			mesh.set_point(myMesh::VertexHandle(*j), p);
			++j;
		}
		mls_point.clear();
	}
	else if (smooth_area == 1){


		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nei(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_nei->width = mls_point.size();
		cloud_nei->height = 1;
		cloud_nei->is_dense = false;
		cloud_nei->points.resize(cloud_nei->width * cloud_nei->height);

		auto j = mls_point.begin();
		//将mls_point中的点存到cloud_nei中
		for (size_t i = 0; i < cloud_nei->points.size(); ++i) {
			cloud_nei->points[i].x = mesh.point(myMesh::VertexHandle(*j))[0];
			cloud_nei->points[i].y = mesh.point(myMesh::VertexHandle(*j))[1];
			cloud_nei->points[i].z = mesh.point(myMesh::VertexHandle(*j))[2];
			++j;
		}
		j = mls_point.begin();

		//mls
		// 创建 KD-Tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		// Output has the PointNormal type in order to store the normals calculated by MLS

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
		// 定义最小二乘实现的对象mls
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

		mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

		// Set parameters
		mls.setInputCloud(cloud_nei);
		mls.setPolynomialOrder(3); //设置false，可以加速平滑
		mls.setSearchMethod(tree);
		mls.setSearchRadius(7); //单位m，设置用于拟合的K近邻半径
		//mls.setNumberOfThreads(4);
		// Reconstruct
		mls.process(*cloud_out); //输出

		for (size_t i = 0; i < cloud_out->points.size(); ++i) {
			myMesh::Point p;
			p[0] = cloud_out->points[i].x;
			p[1] = cloud_out->points[i].y;
			p[2] = cloud_out->points[i].z;
			mesh.set_point(myMesh::VertexHandle(*j), p);
			++j;
		}
		mls_point.clear();
	}


	return;//归一化
}

