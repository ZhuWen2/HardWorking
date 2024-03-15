#include "MeshSmooth.h"
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <minmax.h>
#include <algorithm>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree����������ඨ���ͷ�ļ�
#include <pcl/surface/mls.h>        //��С���˷�ƽ�������ඨ��ͷ�ļ�
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/io/ply_io.h>
typedef OpenMesh::VectorT<float, 3> Vector3;




void GELSmooth::MLSSmooth(myMesh &mesh, int max_iter)
{
	//���������һ������max_iter�Σ�ÿ������GELSmoothFilter����˫���˲���ÿ����ķ��ߵ�����
	//��������filter_normals�С�Ȼ��������λ�ã�����10�ε�����ÿ�ε��������ð�ߵ�����ÿ��
	//��ߵĳ����㶼����һ�飬�������ܱ�֤ÿ�����㱻�����Ĵ�������������ĸ�����
	//����ٰ�mesh�����ж������һ�飬���ղż������ÿ�������λ����Ϣ����������Ĵ���
	//�͵õ���ƽ����λ����Ϣnpos�����Ѵ˶��������nposλ���ϡ�����Ǵ˺����Ĺ�����
	double avg_len = 0.0;//��ƽ������

	int e_it = 0, e_size = mesh.n_edges();
//#pragma omp parallel for default(shared) reduction(+:avg_len)
	for (e_it = 0; e_it < e_size; ++e_it)
	{
		avg_len += mesh.calc_edge_length(myMesh::EdgeHandle(e_it));//Ϊʲô�����еı߳��ȼ�һ��
	}

	std::vector<myMesh::Normal> filter_normals;
	filter_normals.resize(mesh.n_faces(), myMesh::Normal(0.0));
	for (int iter = 0; iter < max_iter; ++iter)//����max_iter��
	{
		int f_it = 0, f_size = mesh.n_faces();//һ���м�����
//#pragma omp parallel for
		for (f_it = 0; f_it < f_size; ++f_it)//ѭ��f_size��
		{
			GELSmoothFilter_MLSpro(mesh, myMesh::FaceHandle(f_it), avg_len / e_size);//����ÿ�����˫���˲���ķ��ߡ�

		}


		mesh.update_face_normals();
	}
}




void GELSmooth::GELSmoothFilter_MLSpro(myMesh & mesh, myMesh::FaceHandle fh, double avg_len)
{

	//�˺����Ǳ���ÿ���棬�����򶥵�MLS������沢����λ�ã�ʵ��ƽ����



	//��������ķ�����filter_normal
	myMesh::Normal filter_normal = myMesh::Normal(0.0, 0.0, 0.0);
	//��������������ķ���������face_neighbor
	std::vector<myMesh::FaceHandle> face_neighbor;
	//������������������face_neighbor��
	getFaceNeighbor(mesh, fh, face_neighbor, true);

	myMesh::Point  pi = mesh.calc_face_centroid(fh);//���������ĵ�
	myMesh::Normal ni = mesh.calc_face_normal(fh);//�����淨��

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
	//����һ����������������ķ��������ȴ����������������֮��ļнǣ�ȡ�н���С�����жϡ�
	std::vector<myMesh::Normal> nei_normal;

	nei_normal.push_back(ni);
	//������Щ����
	std::set<const int> mls_point;
	for (int i = 0; i < face_neighbor.size(); ++i)
	{


		myMesh::FaceHandle f = face_neighbor[i];

		//���smooth_area == 2��������f����face_arris2����ʱ�Ͳ������˲�����
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
			//��ѭ�������������Ķ���
			for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f); fv_it.is_valid(); ++fv_it)
			{
				mls_point.insert((*fv_it).idx());//����������������ŵ�mls_point��
			}
		}

	}



	//if (smooth_area == 2 && neibor_plane < 1) return ni;

	//�����������浽������
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

		//����model���ּ�������
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
	//����mls
	else if (smooth_area==3){
		

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nei(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_nei->width = mls_point.size();
		cloud_nei->height = 1;
		cloud_nei->is_dense = false;
		cloud_nei->points.resize(cloud_nei->width * cloud_nei->height);

		auto j = mls_point.begin();
		//��mls_point�еĵ�浽cloud_nei��
		for (size_t i = 0; i < cloud_nei->points.size(); ++i) {
			cloud_nei->points[i].x = mesh.point(myMesh::VertexHandle(*j))[0];
			cloud_nei->points[i].y = mesh.point(myMesh::VertexHandle(*j))[1];
			cloud_nei->points[i].z = mesh.point(myMesh::VertexHandle(*j))[2];
			++j;
		}
		j = mls_point.begin();

		//mls
		// ���� KD-Tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		// Output has the PointNormal type in order to store the normals calculated by MLS
	
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
		// ������С����ʵ�ֵĶ���mls
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

		mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���

		// Set parameters
		mls.setInputCloud(cloud_nei);
		mls.setPolynomialOrder(3); //����false�����Լ���ƽ��
		mls.setSearchMethod(tree);
		mls.setSearchRadius(7); //��λm������������ϵ�K���ڰ뾶
		//mls.setNumberOfThreads(4);
		// Reconstruct
		mls.process(*cloud_out); //���

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
		//��mls_point�еĵ�浽cloud_nei��
		for (size_t i = 0; i < cloud_nei->points.size(); ++i) {
			cloud_nei->points[i].x = mesh.point(myMesh::VertexHandle(*j))[0];
			cloud_nei->points[i].y = mesh.point(myMesh::VertexHandle(*j))[1];
			cloud_nei->points[i].z = mesh.point(myMesh::VertexHandle(*j))[2];
			++j;
		}
		j = mls_point.begin();

		//mls
		// ���� KD-Tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		// Output has the PointNormal type in order to store the normals calculated by MLS

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
		// ������С����ʵ�ֵĶ���mls
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

		mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���

		// Set parameters
		mls.setInputCloud(cloud_nei);
		mls.setPolynomialOrder(3); //����false�����Լ���ƽ��
		mls.setSearchMethod(tree);
		mls.setSearchRadius(7); //��λm������������ϵ�K���ڰ뾶
		//mls.setNumberOfThreads(4);
		// Reconstruct
		mls.process(*cloud_out); //���

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
		//��mls_point�еĵ�浽cloud_nei��
		for (size_t i = 0; i < cloud_nei->points.size(); ++i) {
			cloud_nei->points[i].x = mesh.point(myMesh::VertexHandle(*j))[0];
			cloud_nei->points[i].y = mesh.point(myMesh::VertexHandle(*j))[1];
			cloud_nei->points[i].z = mesh.point(myMesh::VertexHandle(*j))[2];
			++j;
		}
		j = mls_point.begin();

		//mls
		// ���� KD-Tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		// Output has the PointNormal type in order to store the normals calculated by MLS

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
		// ������С����ʵ�ֵĶ���mls
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

		mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���

		// Set parameters
		mls.setInputCloud(cloud_nei);
		mls.setPolynomialOrder(3); //����false�����Լ���ƽ��
		mls.setSearchMethod(tree);
		mls.setSearchRadius(7); //��λm������������ϵ�K���ڰ뾶
		//mls.setNumberOfThreads(4);
		// Reconstruct
		mls.process(*cloud_out); //���

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


	return;//��һ��
}

