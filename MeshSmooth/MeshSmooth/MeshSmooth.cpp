#include "MeshSmooth.h"
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <minmax.h>
#include <algorithm>
#include <math.h>

typedef OpenMesh::VectorT<float, 3> Vector3;


void DihedralEnergy::compute_angles(const myMesh & m, myMesh::HalfedgeHandle h) const
{
	myMesh::HalfedgeHandle eh = m.opposite_halfedge_handle(h);
	myMesh::VertexHandle hv = m.to_vertex_handle(h);
	myMesh::VertexHandle hov = m.to_vertex_handle(eh);
	myMesh::VertexHandle hnv = m.to_vertex_handle(m.next_halfedge_handle(h));
	myMesh::VertexHandle honv = m.to_vertex_handle(m.next_halfedge_handle(eh));

	OpenMesh::Vec3d va(m.point(hv));
	OpenMesh::Vec3d vb(m.point(hov));
	OpenMesh::Vec3d vc(m.point(hnv));
	OpenMesh::Vec3d vd(m.point(honv));

	myMesh::FaceHandle fa = m.face_handle(m.opposite_halfedge_handle(m.next_halfedge_handle(h)));
	myMesh::FaceHandle fb = m.face_handle(m.opposite_halfedge_handle(m.next_halfedge_handle(m.next_halfedge_handle(h))));
	myMesh::FaceHandle fc = m.face_handle(m.opposite_halfedge_handle(m.next_halfedge_handle(eh)));
	myMesh::FaceHandle fd = m.face_handle(m.opposite_halfedge_handle(m.next_halfedge_handle(m.next_halfedge_handle(eh))));

	OpenMesh::Vec3d n1 = OpenMesh::cross(vc - va, vb - va).normalize();
	OpenMesh::Vec3d n2 = OpenMesh::cross(vb - va, vd - va).normalize();

	OpenMesh::Vec3d na = !m.is_valid_handle(fa) ? OpenMesh::Vec3d(0.0) : OpenMesh::Vec3d(m.normal(fa));
	OpenMesh::Vec3d nb = !m.is_valid_handle(fb) ? OpenMesh::Vec3d(0.0) : OpenMesh::Vec3d(m.normal(fb));
	OpenMesh::Vec3d nc = !m.is_valid_handle(fc) ? OpenMesh::Vec3d(0.0) : OpenMesh::Vec3d(m.normal(fc));
	OpenMesh::Vec3d nd = !m.is_valid_handle(fd) ? OpenMesh::Vec3d(0.0) : OpenMesh::Vec3d(m.normal(fd));

	OpenMesh::Vec3d fn1 = OpenMesh::cross(vb - vc, vd - vc).normalize();
	OpenMesh::Vec3d fn2 = OpenMesh::cross(vd - vc, va - vc).normalize();

	ab_12 = cos_ang(n1, n2);
	ab_a1 = cos_ang(na, n1);
	ab_b1 = cos_ang(nb, n1);
	ab_2c = cos_ang(n2, nc);
	ab_2d = cos_ang(n2, nd);

	aa_12 = cos_ang(fn1, fn2);
	aa_b1 = cos_ang(nb, fn1);
	aa_c1 = cos_ang(nc, fn1);
	aa_2a = cos_ang(fn2, na);
	aa_2d = cos_ang(fn2, nd);
}

double DihedralEnergy::energy(const myMesh & m, myMesh::HalfedgeHandle h) const
{
	myMesh::FaceHandle hf = m.face_handle(h);
	myMesh::FaceHandle hof = m.face_handle(m.opposite_halfedge_handle(h));

	double a = cos_ang(OpenMesh::Vec3d(m.normal(hf)), OpenMesh::Vec3d(m.normal(hof)));

	myMesh::VertexHandle hv = m.to_vertex_handle(h);
	myMesh::VertexHandle hov = m.to_vertex_handle(m.opposite_halfedge_handle(h));

	OpenMesh::Vec3d va(m.point(hv));
	OpenMesh::Vec3d vb(m.point(hov));

	if (use_alpha)
		return edge_alpha_energy(va, vb, a);

	return edge_c_energy(va, vb, a);
}

double DihedralEnergy::delta_energy(const myMesh & m, myMesh::HalfedgeHandle h) const
{
	compute_angles(m, h);

	myMesh::HalfedgeHandle eh = m.opposite_halfedge_handle(h);
	myMesh::VertexHandle hv = m.to_vertex_handle(h);
	myMesh::VertexHandle hov = m.to_vertex_handle(eh);
	myMesh::VertexHandle hnv = m.to_vertex_handle(m.next_halfedge_handle(h));
	myMesh::VertexHandle honv = m.to_vertex_handle(m.next_halfedge_handle(eh));

	OpenMesh::Vec3d va(m.point(hv));
	OpenMesh::Vec3d vb(m.point(hov));
	OpenMesh::Vec3d vc(m.point(hnv));
	OpenMesh::Vec3d vd(m.point(honv));

	if (use_alpha) {
		double before =
			edge_alpha_energy(va, vb, ab_12)
			+ edge_alpha_energy(va, vc, ab_a1)
			+ edge_alpha_energy(vc, vb, ab_b1)
			+ edge_alpha_energy(vd, vb, ab_2c)
			+ edge_alpha_energy(vd, va, ab_2d);

		double after =
			edge_alpha_energy(vd, vc, aa_12)
			+ edge_alpha_energy(vb, vc, aa_b1)
			+ edge_alpha_energy(vd, vb, aa_c1)
			+ edge_alpha_energy(va, vc, aa_2a)
			+ edge_alpha_energy(vd, va, aa_2d);

		return (after - before);
	}
	double before =
		edge_c_energy(va, vb, ab_12)
		+ edge_c_energy(va, vc, ab_a1)
		+ edge_c_energy(vc, vb, ab_b1)
		+ edge_c_energy(vd, vb, ab_2c)
		+ edge_c_energy(vd, va, ab_2d);

	double after =
		edge_c_energy(vd, vc, aa_12)
		+ edge_c_energy(vb, vc, aa_b1)
		+ edge_c_energy(vd, vb, aa_c1)
		+ edge_c_energy(va, vc, aa_2a)
		+ edge_c_energy(vd, va, aa_2d);

	return after - before;
}


void GELSmooth::LaplaceSmooth(myMesh &mesh, int max_iter) {
	int count = 0;
	std::vector<myMesh::Point> vertex_positions(mesh.n_vertices(), myMesh::Point(0.0));
	for (int iter = 0; iter < max_iter; ++iter)//����max_iter��
	{
		int v_it = 0, v_size = mesh.n_vertices();//һ���м�����
		std::cout << v_size << std::endl;
		//�������ж���
		for (v_it = 0; v_it < v_size; ++v_it) {
			
			if (mesh.is_boundary(myMesh::VertexHandle(v_it))){
				vertex_positions[v_it] = mesh.point(myMesh::VertexHandle(v_it));
			}
			else{


				//�ȶ����¼�����º󶥵�λ�õ��������
				double x_new = 0;
				double y_new = 0;
				double z_new = 0;
				double w_sum = 0;
				//����ÿ�������������
				//vertices_face������Ŵ˶����������
				std::vector<myMesh::FaceHandle> vertices_face;

				for (myMesh::VertexFaceIter vf_it = mesh.vf_begin(myMesh::VertexHandle(v_it)); vf_it.is_valid(); ++vf_it)
				{
					//*vf_it���Ǿ��
					myMesh::FaceHandle vf_handle = *vf_it;
					//std::cout << vf_handle.idx() << std::endl;
					vertices_face.push_back(vf_handle);
				}
				//std::cout << vertices_face.size() << std::endl;
				//�������vertices_face��ÿ����İ��
				int i = 0;
				//�ȱ���ÿ����
				for (myMesh::FaceHandle f_it = vertices_face[0]; mesh.is_valid_handle(f_it) && i < vertices_face.size(); ++i) {

					f_it = vertices_face[i];
					//�ٱ�������İ��
					//�ȶ���һ��vector���������߷ֱ�ĳ��ȣ��Ա�����������ֵ��
					std::vector<double> he_length;
					std::vector<double> he_length2;
					for (myMesh::FaceHalfedgeIter fh_it = mesh.fh_begin(f_it); fh_it.is_valid(); ++fh_it) {
						//�����жϴ˰�ߵ��յ��Ƿ��ǸոյĶ���v_it������ǵĻ����ͼ�������ߵĶ���İ�����ڵ����Ƿ���ڣ������������Լ���Ȩ�ء�
						if (mesh.to_vertex_handle(*fh_it) == myMesh::VertexHandle(v_it)) {
							//��ȡ����İ�߾��opp_handle
							myMesh::HalfedgeHandle opp_handle = mesh.opposite_halfedge_handle(*fh_it);
							//��ȡopp_handle������ľ��nei_f
							myMesh::FaceHandle nei_f = mesh.face_handle(opp_handle);
							//std::cout << mesh.is_valid_handle(nei_f) << std::endl;
							//�����������������ڣ�˵�������ߴ��������棬�������Ȩ��
							if (
								(!mesh.is_boundary(f_it)) && (!mesh.is_boundary(nei_f))
								//nei_f.is_valid() && (!mesh.is_boundary(opp_handle))
								) {
								//�ȴ�ӡһ��������Ķ���
								/*for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f_it); fv_it.is_valid(); ++fv_it)
								{
								myMesh::Point p1 =  mesh.point(*fv_it);
								std::cout << p1[0] << "  " << p1[1] << "  " << p1[2] << "|";
								}
								std::cout << std::endl;
								for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(nei_f); fv_it.is_valid(); ++fv_it)
								{
								myMesh::Point p1 = mesh.point(*fv_it);
								std::cout << p1[0] << "  " << p1[1] << "  " << p1[2] << "|";
								}*/
								//std::cout << std::endl;

								//������������ߵĳ���
								//���������ߵĳ���
								myMesh::Point from_point = mesh.point(mesh.from_vertex_handle(*fh_it));
								myMesh::Point to_point = mesh.point(mesh.to_vertex_handle(*fh_it));
								double x = from_point[0] - to_point[0];
								//std::cout << x << std::endl;
								double y = from_point[1] - to_point[1];
								double z = from_point[2] - to_point[2];
								double len = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
								he_length.push_back(len);
								//std::cout << len << "  ";

								//������һ����߳���
								myMesh::HalfedgeHandle next_he1 = mesh.next_halfedge_handle(*fh_it);
								from_point = mesh.point(mesh.from_vertex_handle(next_he1));
								to_point = mesh.point(mesh.to_vertex_handle(next_he1));
								x = from_point[0] - to_point[0];
								y = from_point[1] - to_point[1];
								z = from_point[2] - to_point[2];
								len = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
								he_length.push_back(len);
								//std::cout << len << "  ";
								//������һ����߳���
								myMesh::HalfedgeHandle next_he2 = mesh.next_halfedge_handle(next_he1);
								from_point = mesh.point(mesh.from_vertex_handle(next_he2));
								to_point = mesh.point(mesh.to_vertex_handle(next_he2));
								x = from_point[0] - to_point[0];
								y = from_point[1] - to_point[1];
								z = from_point[2] - to_point[2];
								len = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
								he_length.push_back(len);
								//std::cout << len << "  ";
								//�ټ�������������߳���

								from_point = mesh.point(mesh.from_vertex_handle(opp_handle));
								to_point = mesh.point(mesh.to_vertex_handle(opp_handle));
								x = from_point[0] - to_point[0];
								y = from_point[1] - to_point[1];
								z = from_point[2] - to_point[2];
								len = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
								he_length2.push_back(len);

								next_he1 = mesh.next_halfedge_handle(opp_handle);
								from_point = mesh.point(mesh.from_vertex_handle(next_he1));
								to_point = mesh.point(mesh.to_vertex_handle(next_he1));
								x = from_point[0] - to_point[0];
								y = from_point[1] - to_point[1];
								z = from_point[2] - to_point[2];
								len = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
								he_length2.push_back(len);

								next_he2 = mesh.next_halfedge_handle(next_he1);
								from_point = mesh.point(mesh.from_vertex_handle(next_he2));
								to_point = mesh.point(mesh.to_vertex_handle(next_he2));
								x = from_point[0] - to_point[0];
								y = from_point[1] - to_point[1];
								z = from_point[2] - to_point[2];
								len = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
								he_length2.push_back(len);

								/*std::cout << he_length[0] << "  " << he_length[1] << "  " << he_length[2] << std::endl;
								std::cout << he_length2[0] << "  " << he_length2[1] << "  " << he_length2[2] << std::endl;*/
								//������������Ƕ�
								double cosa = (he_length[1] * he_length[1] + he_length[2] * he_length[2] - he_length[0] * he_length[0]) / (2 * he_length[1] * he_length[2]);

								double a = acos(cosa);//������
								double cosb = (he_length2[1] * he_length2[1] + he_length2[2] * he_length2[2] - he_length2[0] * he_length2[0]) / (2 * he_length2[1] * he_length2[2]);
								double b = acos(cosb);
								//std::cout << cosa << "  " << cosb << "  ";
								//std::cout << a << "  " << b << "  ";
								//������
								double cota = 1 / tan(a);
								double cotb = 1 / tan(b);

								//����任��ĵ�����
								double w = (cota + cotb) / 2;
								myMesh::Point pp = mesh.point(mesh.from_vertex_handle(*fh_it));
								myMesh::Point p = mesh.point(mesh.to_vertex_handle(*fh_it));
								x_new += w*(pp[0] - p[0]);
								y_new += w*(pp[1] - p[1]);
								z_new += w*(pp[2] - p[2]);
								w_sum += w;

								

								he_length.clear();
								he_length2.clear();

								//std::cout << cota << "  " << cotb << std::endl;
							}


						}
					}

				}
				//����ľ���̫��Ĳ���λ��
				if (sqrt(pow((x_new / w_sum), 2) + pow((y_new / w_sum), 2) + pow((z_new / w_sum), 2)) > 10){
					/*for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f_it); fv_it.is_valid(); ++fv_it)
					{
					myMesh::Point p1 =  mesh.point(*fv_it);
					std::cout << p1[0] << "  " << p1[1] << "  " << p1[2] << "|";
					}
					std::cout << std::endl;
					for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(nei_f); fv_it.is_valid(); ++fv_it)
					{
					myMesh::Point p1 = mesh.point(*fv_it);
					std::cout << p1[0] << "  " << p1[1] << "  " << p1[2] << "|";
					}
					std::cout << std::endl;
					std::cout << he_length[0] << "  " << he_length[1] << "  " << he_length[2] << std::endl;
					std::cout << he_length2[0] << "  " << he_length2[1] << "  " << he_length2[2] << std::endl;*/
					x_new = 0;
					y_new = 0;
					z_new = 0;
				}
				//��ʱ�Ѿ�������һ������������棬�����ټ����µĶ�������
				myMesh::Point p_new = mesh.point(myMesh::VertexHandle(v_it));
				//�������0��˵���õ�û���㣬�ǾͲ��ı�λ��
				if (x_new != 0) p_new[0] = mesh.point(myMesh::VertexHandle(v_it))[0] + x_new / w_sum;
				if (y_new != 0) p_new[1] = mesh.point(myMesh::VertexHandle(v_it))[1] + y_new / w_sum;
				if (z_new != 0) p_new[2] = mesh.point(myMesh::VertexHandle(v_it))[2] + z_new / w_sum;
				//��������Ҫ�ı���λ�õ�p_new��
				vertex_positions[v_it] = p_new;
				//mesh.set_point(myMesh::VertexHandle(v_it), p_new);
				/*std::cout << mesh.point(myMesh::VertexHandle(v_it))[0] << "  " << mesh.point(myMesh::VertexHandle(v_it))[1] << "  " << mesh.point(myMesh::VertexHandle(v_it))[2] << "  ";
				std::cout << p_new[0] << "  " << p_new[1] << "  " << p_new[2]<< std::endl;*/
				
			}

		
		}

		for (int i = 0; i < vertex_positions.size(); ++i) {
			mesh.set_point(myMesh::VertexHandle(i), vertex_positions[i]);
		}
		vertex_positions.clear();

	}
	
}

void GELSmooth::denoise(myMesh &mesh, int max_iter)
{	
	//���������һ������max_iter�Σ�ÿ������GELSmoothFilter����˫���˲���ÿ����ķ��ߵ�����
	//��������filter_normals�С�Ȼ��������λ�ã�����10�ε�����ÿ�ε��������ð�ߵ�����ÿ��
	//��ߵĳ����㶼����һ�飬�������ܱ�֤ÿ�����㱻�����Ĵ�������������ĸ�����
	//����ٰ�mesh�����ж������һ�飬���ղż������ÿ�������λ����Ϣ����������Ĵ���
	//�͵õ���ƽ����λ����Ϣnpos�����Ѵ˶��������nposλ���ϡ�����Ǵ˺����Ĺ�����
	double avg_len = 0.0;//��ƽ������

	int e_it = 0, e_size = mesh.n_edges();
#pragma omp parallel for default(shared) reduction(+:avg_len)
	for (e_it = 0; e_it < e_size; ++e_it)
	{
		avg_len += mesh.calc_edge_length(myMesh::EdgeHandle(e_it));//Ϊʲô�����еı߳��ȼ�һ��
	}

	std::vector<myMesh::Normal> filter_normals;
	filter_normals.resize(mesh.n_faces(), myMesh::Normal(0.0));
	for (int iter = 0; iter < max_iter; ++iter)//����max_iter��
	{
		int f_it = 0, f_size = mesh.n_faces();//һ���м�����
#pragma omp parallel for
		for (f_it = 0; f_it < f_size; ++f_it)//ѭ��f_size��
		{
			filter_normals[f_it] = GELSmoothFilter_pro(mesh, myMesh::FaceHandle(f_it), avg_len / e_size);//����ÿ�����˫���˲���ķ��ߡ�
			
		}

		//���������λ��
		//����������ʿ���ĵ�˫���˲��������������
		std::vector<myMesh::Point> vertex_positions(mesh.n_vertices(), myMesh::Point(0.0));
		std::vector<int> count(mesh.n_vertices(), 0);
		for (int sub_iter = 0; sub_iter < 10; ++sub_iter)//Ϊʲô��10��ѭ��
		{	
			//ÿ�α������а�ߣ�����ÿ����ߵ��淨�ߺʹ˰�ߵĹ�ϵ�����˰�ߵĳ������㣬
			//����¼ÿ����������˼���
			//�����forѭ������ÿ����ߣ�ÿ��������������ߣ����ÿ�����㱻�����Ĵ������Ƕ������ڵ���ĸ���
			for (myMesh::HalfedgeIter he_it = mesh.halfedges_begin(); he_it != mesh.halfedges_end(); ++he_it)
			{
				//��ȡhe_it������ڵ���fh
				myMesh::FaceHandle fh = mesh.face_handle(*he_it);
				if (fh.is_valid())
				{
					myMesh::VertexHandle vh = mesh.from_vertex_handle(*he_it);
					//std::cout << vh << std::endl;
					//�����һ�����
					myMesh::HalfedgeHandle prev_heit = mesh.prev_halfedge_handle(*he_it);
					myMesh::Normal dir2 = mesh.point(mesh.to_vertex_handle(prev_heit)) - mesh.point(vh);
					myMesh::Normal dir = mesh.point(mesh.to_vertex_handle(*he_it)) - mesh.point(vh);
					
					myMesh::Normal n = filter_normals[fh.idx()];
					myMesh::Normal Vi = n * (OpenMesh::dot(n, dir) + OpenMesh::dot(n, dir2));
					//vertex_positions[vh.idx()] += mesh.point(vh) + 0.5 * n * OpenMesh::dot(n, dir);
					vertex_positions[vh.idx()] += mesh.point(vh) + 0.33 * Vi;
					count[vh.idx()] += 1;//��¼ÿ����������˼��Σ����Ӧ�ô����Ƕ������ڵ���ĸ�����



					//myMesh::VertexHandle vh = mesh.from_vertex_handle(*he_it);
					//myMesh::Normal dir = mesh.point(mesh.to_vertex_handle(*he_it)) - mesh.point(vh);
					//myMesh::Normal n = filter_normals[fh.idx()];
					//vertex_positions[vh.idx()] += mesh.point(vh) + 0.5 * n * OpenMesh::dot(n, dir);
					//count[vh.idx()] += 1;//��¼ÿ����������˼��Σ����Ӧ�ô����Ƕ������ڵ���ĸ�����

				}
			}

			double max_move = 0;
			int v_it = 0, v_size = mesh.n_vertices();//����ĸ���
#pragma omp parallel for//�����ѭ���������������vertex_positions����count[v_it]�õ�ƽ����λ��npos��
						//��������nposλ���ϡ�
			for (v_it = 0; v_it < v_size; ++v_it)//ѭ��v_size��
			{
				myMesh::Point npos;
				int j = v_it;
				if (count[j] == 0)//Ϊʲô���ж���û����������
					npos = mesh.point(myMesh::VertexHandle(v_it));
				else
				{
					npos = vertex_positions[v_it] / double(count[v_it]);
					myMesh::Point a = npos - mesh.point(myMesh::VertexHandle(v_it));
					double move = OpenMesh::dot(a, a);

					//#pragma omp critical
					if (move > max_move)
						max_move = move;

				}
				mesh.set_point(myMesh::VertexHandle(v_it), npos);//��v_it���ڶ��������npos��λ����ȥ
			}
			//��������ƶ����붼�Ѿ�С����ֵ���Ͳ����ٵ����ˣ�ֱ��break�Ƴ�ѭ����
			if (max_move < (1e-8*avg_len)*(1e-8*avg_len))
			{
				break;
			}
		}

		mesh.update_face_normals();
	}
}

myMesh::Normal GELSmooth::GELSmoothFilter(myMesh & mesh, myMesh::FaceHandle fh, double avg_len)
{
	//�˺����ǰ�����Ϊfh���������ڽ����ҳ���Ȼ������ڽ��沢������Щ�ڽ������Ϣ���뵽˫���˲���ʽ�У����õ�˫���˲���Ĵ���ķ�������

	//��������ķ�����filter_normal
	myMesh::Normal filter_normal = myMesh::Normal(0.0, 0.0, 0.0);
	//��������������ķ���������face_neighbor
	std::vector<myMesh::FaceHandle> face_neighbor;
	//���������淨��������face_neighbor��
	getFaceNeighbor(mesh, fh, face_neighbor, true);
	myMesh::Point  pi = mesh.calc_face_centroid(fh);//���������ĵ�
	myMesh::Normal ni = mesh.calc_face_normal(fh);//�����淨��

	//������Щ����
	for (int i = 0; i < face_neighbor.size(); ++i)
	{
		myMesh::FaceHandle f = face_neighbor[i];
		myMesh::Normal nj = mesh.calc_face_normal(f);
		
		myMesh::Point  pj = mesh.calc_face_centroid(f);
		//�����Ǽ���Wa��Ws������˫�߷����˲����������˹������
		double w_a = std::exp(-std::acos(max(-1.0, min(1.0, OpenMesh::dot(nj, ni)))) / (M_PI / 32));
		double w_s = std::exp(-(pj - pi).length() / avg_len);

		myMesh::Point face_vertices[3];
		int j = 0;
		//��ѭ�������������Ķ���
		for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f); fv_it.is_valid(); ++fv_it)
		{
			face_vertices[j++] = mesh.point(*fv_it);//����������������ŵ�face_vertices��
		}
		myMesh::Normal e1 = face_vertices[1] - face_vertices[0];
		myMesh::Normal e2 = face_vertices[2] - face_vertices[0];//���Ǽ���������e1,e2
		double area = 0.5 * OpenMesh::cross(e1, e2).length();//�������������
		filter_normal += area * w_a*w_s*nj;//����˫���˲���ʽ������ͣ�����һ����
	}

	return filter_normal.normalize();//��һ��
}


myMesh::Normal GELSmooth::GELSmoothFilter_pro(myMesh & mesh, myMesh::FaceHandle fh, double avg_len)
{
	
	//�˺����ǰ�����Ϊfh���������ڽ����ҳ���Ȼ������ڽ��沢������Щ�ڽ������Ϣ���뵽˫���˲���ʽ�У����õ�˫���˲���Ĵ���ķ�������

	

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
			return ni;
		}
	}
	if (smooth_area == 3) {
		if (face_arris2.find(fh.idx()) == face_arris2.end()) {
			return ni;
		}
	}
	if (smooth_area == 4) {
		if (face_arris3.find(fh.idx()) == face_arris3.end()) {
			return ni;
		}
	}


	int count1 = 0;
	int count2 = 0;
	//����һ����������������ķ��������ȴ����������������֮��ļнǣ�ȡ�н���С�����жϡ�
	std::vector<myMesh::Normal> nei_normal;
	
	nei_normal.push_back(ni);
	//������Щ����
	//int neibor_plane = 0;
	for (int i = 0; i < face_neighbor.size(); ++i)
	{	


		myMesh::FaceHandle f = face_neighbor[i];

		//���smooth_area == 3��������f����face_arris2����ʱ�Ͳ�����˫���˲�����
		/*if (smooth_area == 2){
			if (face_arris1.find(f.idx()) == face_arris1.end()) {
				continue;
			}

		}*/
		/*if (smooth_area == 3){
			if (face_arris2.find(f.idx()) == face_arris2.end()) {
				continue;
			}
			
		}*/
		if (smooth_area == 4){
			if (face_arris3.find(f.idx()) == face_arris3.end()) {
				continue;
			}

		}
		


		myMesh::Normal nj = mesh.calc_face_normal(f);
		
		myMesh::Point  pj = mesh.calc_face_centroid(f);

		double length1 = nj.length();
		double length2 = ni.length();
		//std::cout << length2 << "  ";
		double theta = 180 / 3.14159*acos(OpenMesh::dot(nj, ni) / (length1*length2));
		if (smooth_area == 1) {
			//�����Ǽ���Wa��Ws������˫�߷����˲����������˹������
			double w_a = std::exp(-std::acos(max(-1.0, min(1.0, OpenMesh::dot(nj, ni)))) / (M_PI / 32));
			double w_s = std::exp(-(pj - pi).length() / avg_len);

			myMesh::Point face_vertices[3];
			int j = 0;
			//��ѭ�������������Ķ���
			for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f); fv_it.is_valid(); ++fv_it)
			{
				face_vertices[j++] = mesh.point(*fv_it);//����������������ŵ�face_vertices��
			}
			myMesh::Normal e1 = face_vertices[1] - face_vertices[0];
			myMesh::Normal e2 = face_vertices[2] - face_vertices[0];//���Ǽ���������e1,e2
			double area = 0.5 * OpenMesh::cross(e1, e2).length();//�������������
			filter_normal += area * w_a*w_s*nj;//����˫���˲���ʽ������ͣ�����һ����
		}
		
		else if (smooth_area == 2){
			
			//�����Ǽ���Wa��Ws������˫�߷����˲����������˹������
			double w_a = std::exp(-std::acos(max(-1.0, min(1.0, OpenMesh::dot(nj, ni)))) / (M_PI / 32));
			double w_s = std::exp(-(pj - pi).length() / avg_len);

			myMesh::Point face_vertices[3];
			int j = 0;
			//��ѭ�������������Ķ���
			for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f); fv_it.is_valid(); ++fv_it)
			{
				face_vertices[j++] = mesh.point(*fv_it);//����������������ŵ�face_vertices��
			}
			myMesh::Normal e1 = face_vertices[1] - face_vertices[0];
			myMesh::Normal e2 = face_vertices[2] - face_vertices[0];//���Ǽ���������e1,e2
			double area = 0.5 * OpenMesh::cross(e1, e2).length();//�������������
			filter_normal += area * w_a*w_s*nj;//����˫���˲���ʽ������ͣ�����һ����

			
		}
		
		else if (smooth_area == 3){
			//�����Ǽ���Wa��Ws������˫�߷����˲����������˹������
			double w_a = std::exp(-std::acos(max(-1.0, min(1.0, OpenMesh::dot(nj, ni)))) / (M_PI / 32));
			double w_s = std::exp(-(pj - pi).length() / avg_len);

			myMesh::Point face_vertices[3];
			int j = 0;
			//��ѭ�������������Ķ���
			for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f); fv_it.is_valid(); ++fv_it)
			{
				face_vertices[j++] = mesh.point(*fv_it);//����������������ŵ�face_vertices��
			}
			myMesh::Normal e1 = face_vertices[1] - face_vertices[0];
			myMesh::Normal e2 = face_vertices[2] - face_vertices[0];//���Ǽ���������e1,e2
			double area = 0.5 * OpenMesh::cross(e1, e2).length();//�������������
			filter_normal += area * w_a*w_s*nj;//����˫���˲���ʽ������ͣ�����һ����


		}
		else if (smooth_area == 4){
			//�����Ǽ���Wa��Ws������˫�߷����˲����������˹������
			double w_a = std::exp(-std::acos(max(-1.0, min(1.0, OpenMesh::dot(nj, ni)))) / (M_PI / 32));
			double w_s = std::exp(-(pj - pi).length() / avg_len);

			myMesh::Point face_vertices[3];
			int j = 0;
			//��ѭ�������������Ķ���
			for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f); fv_it.is_valid(); ++fv_it)
			{
				face_vertices[j++] = mesh.point(*fv_it);//����������������ŵ�face_vertices��
			}
			myMesh::Normal e1 = face_vertices[1] - face_vertices[0];
			myMesh::Normal e2 = face_vertices[2] - face_vertices[0];//���Ǽ���������e1,e2
			double area = 0.5 * OpenMesh::cross(e1, e2).length();//�������������
			filter_normal += area * w_a*w_s*nj;//����˫���˲���ʽ������ͣ�����һ����


		}
		else if (smooth_area == 0) {
			nei_normal.push_back(nj);
			
		}
		

	}

	//if (smooth_area == 2 && neibor_plane < 1) return ni;
	
	//�����������浽������
	if (smooth_area == 0){
		for (int i = 0; i<nei_normal.size()-1; ++i) {
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
				return ni;

			}
			else {
				face_arris2.insert(fh.idx());
				return ni;
			}
			nei_normal.clear();
		}
		else if (model == 3){
			if (double(count1) / hhhh >= 0.01) {
				face_arris1.insert(fh.idx());
				return ni;

			}
			else if (double(count2) / hhhh >= 0.01){
				face_arris2.insert(fh.idx());
				return ni;
			}
			else {
				face_arris3.insert(fh.idx());
				return ni;
			}
			nei_normal.clear();
		}
		
	}
	
	
	


	////double theta = 40;
	//int count_theta = 0;
	////����ÿ��������ķ�������淨�ߵļн�
	//for (int i = 0; i < face_normal.size(); ++i) {
	//	double length1 = face_normal[i].length();
	//	double length2 = ni.length();
	//	//std::cout << length2 << "  ";
	//	double theta = 180 / 3.14159*acos(OpenMesh::dot(face_normal[i], ni) / (length1*length2));
	//	//std::cout << theta << "  ";
	//	//if (smooth_area == 1){
	//	//	if (theta>35) {
	//	//		++count_theta;
	//	//		if (double(count_theta) / face_normal.size() >= 0.1){
	//	//			face_normal.clear();
	//	//			//std::cout << "hh"<<std::endl;
	//	//			return ni;
	//	//		}
	//	//	}
	//	//}
	//	
	//

	//}
	
	


	return filter_normal.normalize();//��һ��
}

void GELSmooth::getFaceNeighbor(myMesh & mesh, myMesh::FaceHandle fh, std::vector<myMesh::FaceHandle>& face_neighbor, bool is_include_self)
{
	face_neighbor.clear();
	if (is_include_self)//�������������true����ô�Ͱ�������������mesh������������fh�Ž�������
	{
		face_neighbor.push_back(fh);
	}
	
	//������fh��һ����Ȧ��Ƭ
	
	//��������ѭ����ѭ��ÿ����Ķ���
	for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(fh); fv_it.is_valid(); ++fv_it)
	{
		//�������ѭ������ѭ����ѭ��ÿ�������������
		for (myMesh::VertexFaceIter vf_it = mesh.vf_iter(*fv_it); vf_it.is_valid(); ++vf_it)
		{
			if ((*vf_it) != fh) {
				face_neighbor.push_back(*vf_it);
				if (radius >= 2){
					for (myMesh::FaceVertexIter fv_it1 = mesh.fv_begin(*vf_it); fv_it1.is_valid(); ++fv_it1) {
						for (myMesh::VertexFaceIter vf_it1 = mesh.vf_iter(*fv_it1); vf_it1.is_valid(); ++vf_it1) {
							if ((*vf_it1) != *vf_it) {
								face_neighbor.push_back(*vf_it1);
								if (radius >= 3){
									for (myMesh::FaceVertexIter fv_it2 = mesh.fv_begin(*vf_it1); fv_it2.is_valid(); ++fv_it2) {
										for (myMesh::VertexFaceIter vf_it2 = mesh.vf_iter(*fv_it2); vf_it2.is_valid(); ++vf_it2) {
											if ((*vf_it2) != *vf_it1) {
												face_neighbor.push_back(*vf_it2);

											}
										}
									}
								}
							}
						}
					}
				}
			}//������Ǵ�������fh����棬�Ͱ���push��face_neighbor�С�//������ʱû�����ظ������⣬��Ϊ�����ȥ���ظ��������ȥ�ط�����ʦ��˵���ֱ�Ӵ����set�п졣
				
		}
	}
	//std::unique�����������ǡ�ȥ����������������������Ԫ�ص��ظ����ֵ�Ԫ�أ�ע�� 
	//(1) �����ȥ���������������erase�����ǽ��ظ���Ԫ�طŵ�������ĩβ������ֵ��ȥ��֮���β��ַ��
	//(2) unique��Ե�������Ԫ�أ����Զ���˳��˳����ҵ������Ա������������Ա����Ҫ�Ƚ������򣬿��Ե���std::sort()����
	std::sort(face_neighbor.begin(), face_neighbor.end());//��������
	face_neighbor.erase(std::unique(face_neighbor.begin(), face_neighbor.end()), face_neighbor.end());
	//���������˼�ǰ�std::unique���ص�β��ֱַ��face_neighbor.end()��ɾ������
	
}

void GELSmooth::optimize_mesh(myMesh& m)
{
	DihedralEnergy efun(4, true);
	priority_queue_optimization(m, efun);
}

void GELSmooth::priority_queue_optimization(myMesh & m, const EnergyFun & efun)
{
	std::vector<HalfEdgeCounter> counter(m.n_halfedges(), HalfEdgeCounter{ 0, false });
	std::vector<int> flipCounter(m.n_vertices(), 0);
	std::priority_queue<PQElement> Q;

	int time = 1;
	for (myMesh::HalfedgeIter he_it = m.halfedges_begin(); he_it != m.halfedges_end(); ++he_it)
	{
		if (!counter[he_it->idx()].touched) {
			add_to_queue(m, counter, Q, *he_it, efun, flipCounter, time);
		}
	}

	while (!Q.empty())
	{
		PQElement elem = Q.top();
		Q.pop();

		if (counter[elem.h.idx()].isRemovedFromQueue)
			continue;
		counter[elem.h.idx()].isRemovedFromQueue = true;

		if (counter[elem.h.idx()].touched != elem.time)
			continue;

		if (efun.delta_energy(m, elem.h) >= -0.001)
			continue;

		if (!m.is_flip_ok(m.edge_handle(elem.h)))
			continue;

		flipCounter[m.to_vertex_handle(elem.h).idx()]++;

		m.flip(m.edge_handle(elem.h));

		add_one_ring_to_queue(m, counter, Q, m.to_vertex_handle(elem.h), efun, flipCounter, time);
		add_one_ring_to_queue(m, counter, Q, m.to_vertex_handle(m.next_halfedge_handle(elem.h)), efun, flipCounter, time);
		add_one_ring_to_queue(m, counter, Q, m.to_vertex_handle(m.opposite_halfedge_handle(elem.h)), efun, flipCounter, time);
		add_one_ring_to_queue(m, counter, Q, m.to_vertex_handle(m.next_halfedge_handle(m.opposite_halfedge_handle(elem.h))), efun, flipCounter, time);
	}

}

void GELSmooth::add_to_queue(const myMesh & m, std::vector<HalfEdgeCounter>& counter, std::priority_queue<PQElement>& Q, myMesh::HalfedgeHandle h, const EnergyFun & efun, std::vector<int>& flipCounter, int time)
{
	if (m.is_boundary(h))
		return;

	if (m.to_vertex_handle(h).idx() < m.from_vertex_handle(h).idx())
		h = m.opposite_halfedge_handle(h);

	if (counter[h.idx()].touched == time)
		return;
	counter[h.idx()].isRemovedFromQueue = false;

	if (!m.is_flip_ok(m.edge_handle(h)))
		return;

	double energy = efun.delta_energy(m, h);
	counter[h.idx()].touched = time;

	const int avgValence = 6;
	if ((energy < 0) && (flipCounter[m.to_vertex_handle(h).idx()] < avgValence))
		Q.push(PQElement(energy, h, time));

}

void GELSmooth::add_one_ring_to_queue(const myMesh & m, std::vector<HalfEdgeCounter>& touched, std::priority_queue<PQElement>& Q, myMesh::VertexHandle v, const EnergyFun & efun, std::vector<int>& flipCounter, int time)
{
	for (myMesh::ConstVertexVertexIter vv_it = m.cvv_iter(v); vv_it.is_valid(); ++vv_it)
	{
		add_to_queue(m, touched, Q, m.halfedge_handle(v), efun, flipCounter, time);
	}
}

