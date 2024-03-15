#include "MeshSmooth.h"
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <minmax.h>
#include <algorithm>
#include <math.h>

typedef OpenMesh::VectorT<float, 3> Vector3;



void GELSmooth::Sharp(myMesh &mesh, int max_iter, int &count_s, std::vector<std::vector<const int> >& sharp_face_neiidx)
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
		avg_len += mesh.calc_edge_length(myMesh::EdgeHandle(e_it));
	}

	std::vector<myMesh::Normal> filter_normals;
	filter_normals.resize(mesh.n_faces(), myMesh::Normal(0.0));
	for (int iter = 0; iter < max_iter; ++iter)//����max_iter��
	{
		int f_it = 0, f_size = mesh.n_faces();//һ���м�����
		//#pragma omp parallel for
		//�ȱ���һ��������Ƭ���ҵ�������Ƭ�����浽map�ͬʱ�Ѹ��Ե�������Ƭ���浽��ά������
		for (f_it = 0; f_it < f_size; ++f_it)//ѭ��f_size��
		{
			SearchFeatures(mesh, myMesh::FaceHandle(f_it), count_s, sharp_face_neiidx);

		}
		//��������Ĺ����Ǳ����ղ��ҵ���������Ƭ������Ȩֵ���Ѽ���õ�����Ƭ�����ͷ������ŵ�һ���µ�map��
		CalculateNormal(mesh, myMesh::FaceHandle(f_it), sharp_face_neiidx);
		for (f_it = 0; f_it < f_size; ++f_it)//ѭ��f_size��
		{
			filter_normals[f_it] = GetCalculateNormal(mesh, myMesh::FaceHandle(f_it), avg_len);
			/*if (f_it<10)
			std::cout << filter_normals[f_it]<<"  " << std::endl;*/

		}
		std::cout << std::endl;
		sharp_face.clear();
		sharp_face_normal.clear();
		while (!pq.empty()) pq.pop();
		count_s = 0;
		radius = 1;
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
		
							myMesh::Point  ck = mesh.calc_face_centroid(fh);//���������ĵ�
							myMesh::Normal nk = filter_normals[fh.idx()];
							myMesh::Normal nn = nk*OpenMesh::dot(nk, ck - mesh.point(vh));
							vertex_positions[vh.idx()] += mesh.point(vh) + nn;
							//vertex_positions[vh.idx()] += mesh.point(vh) + 0.33 * Vi;
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
		//#pragma omp parallel for//�����ѭ���������������vertex_positions����count[v_it]�õ�ƽ����λ��npos��
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
		filter_normals.clear();
		mesh.update_face_normals();//�����淨����Ϣ
	}
}

//Ѱ������
void GELSmooth::SearchFeatures(myMesh & mesh, myMesh::FaceHandle fh, int &count_s, std::vector<std::vector<const int> >& sharp_face_neiidx)
{

	//�˺����ǰ�����Ϊfh���������ڽ����ҳ���Ȼ������ڽ��沢������Щ�ڽ������Ϣ���뵽˫���˲���ʽ�У����õ�˫���˲���Ĵ���ķ�������



	//��������ķ�����filter_normal
	myMesh::Normal filter_normal = myMesh::Normal(0.0, 0.0, 0.0);
	//��������������ķ���������face_neighbor
	std::vector<myMesh::FaceHandle> face_neighbor;
	//������������������face_neighbor��
	getFaceNeighbor_sharp(mesh, fh, face_neighbor, true);
	myMesh::Normal ni = mesh.calc_face_normal(fh);//�����淨��


	int count1 = 0;

	//����һ����������������ķ��������ȴ����������������֮��ļнǣ�ȡ�н���С�����жϡ�
	std::vector<myMesh::Normal> nei_normal;
	std::vector<const int> nei_idx;
	nei_normal.push_back(ni);
	nei_idx.push_back(fh.idx());
	//������Щ����

	for (int i = 0; i < face_neighbor.size(); ++i)
	{
		myMesh::FaceHandle f = face_neighbor[i];
		myMesh::Normal nj = mesh.calc_face_normal(f);

		double length1 = ni.length();
		double length2 = nj.length();
		double theta = 180 / 3.14159*acos(OpenMesh::dot(ni, nj) / (length1*length2));
		if (theta < 90){
			nei_normal.push_back(nj);
			nei_idx.push_back(f.idx());
		}
	}


	//�����������浽������

	for (int i = 0; i < nei_normal.size() - 1; ++i) {
		for (int j = i + 1; j < nei_normal.size(); ++j) {
			double length1 = nei_normal[i].length();
			double length2 = nei_normal[j].length();
			double theta = 180 / 3.14159*acos(OpenMesh::dot(nei_normal[j], nei_normal[i]) / (length1*length2));
			if (theta > min_angle) {
				++count1;
			}

		}
	}

	int hhhh = nei_normal.size()*(nei_normal.size() - 1) / 2;


	if (double(count1) / hhhh >= 0.01) {

		sharp_face.insert(std::pair<const int, int>(fh.idx(), count_s));
		sharp_face_neiidx.push_back(nei_idx);
		//std::cout << sharp_face_neiidx[count_s].size() << std::endl;
		++count_s;
		//std::cout << count_s << std::endl;
		//sharp_face.insert(fh.idx());
	}
	nei_normal.clear();
	nei_idx.clear();



}




//��������ȱ������������棬����Ȩ�أ�����Χ����������ȱ�������������Ȼ�������Ƭ�Ķ������������Ҫע���������������ĵķ����������������㶼��
//�������������������������Լ�����һ���Ķ�������������������ڵĶ��㲻�ᱻ�����������ͱ��������ݵ����⡣�����ͬʱ��Ҳ����ÿ������һ��������Ƭ��
//�ĳɷ������棬��Ϊ������Ӱ�쵽�Աߵ���Ƭ�������Աߵ���Ƭ�������㶼��������������ڣ�����Ͳ���������ˡ�����Ҫ�����ȶ����е�Ȩ�ز�Ϊ��ģ���
//������������ڵ�������Ƭȫ��������֮��������Ϊ�������档
//���������������Ĳ�ͬ
//1.������������ڵĶ��㲻����λ��
//2.�����������������Ϊ���������ʱ����ͬ��
void GELSmooth::CalculateNormal(myMesh & mesh, myMesh::FaceHandle fh, std::vector<std::vector<const int> >& sharp_face_neiidx)
{
	int max_t = 0;
	do {
		//std::cout << pq.size() << std::endl;
		while (!pq.empty()) pq.pop();
		//����������Ƭ
		for (auto it_fe = sharp_face.cbegin(); it_fe != sharp_face.cend(); ++it_fe) {
			//���Ȩֵ
			double max_w = 0;

			//������Ƭfw����Ϊ����
			myMesh::FaceHandle fw = myMesh::FaceHandle((*it_fe).first);
			double w = 0;
			int neiidx = (*it_fe).second;
			//�������������Ƭ��������
			for (int i = 0; i < sharp_face_neiidx[neiidx].size(); ++i) {
				//ÿ��������idx��Ϊ n
				const int n = sharp_face_neiidx[neiidx][i];

				//������������ڷ�������
				if (sharp_face.find(n) == sharp_face.end()) {
					myMesh::Normal ni = mesh.calc_face_normal(myMesh::FaceHandle((*it_fe).first));//�����淨��
					myMesh::Normal nj = mesh.calc_face_normal(myMesh::FaceHandle(n));//�����淨��
					w = OpenMesh::dot(ni, nj);
					if (w > max_w) {
						max_w = w;

						fw = myMesh::FaceHandle(n);

					}

					if (w < 0) w = 0;


				}

			}
			//std::cout <<  std::endl;
			//������������֮�󣬰�Ȩֵ���������뵽���ȶ���
			std::pair<myMesh::FaceHandle, myMesh::FaceHandle> p = std::make_pair(myMesh::FaceHandle((*it_fe).first), fw);
			pq.push(std::make_pair(w, p));


		}
		std::vector<const int> v_idx;
		//Ȼ�����γ��Ӽ�����µķ�����
		while (pq.top().first){
			//������º�ķ�����nf
			myMesh::Normal nf = myMesh::Normal(0.0, 0.0, 0.0);
			//������Ƭf������������Ƭfw
			myMesh::FaceHandle f = pq.top().second.first;

			myMesh::Normal nn = mesh.calc_face_normal(f);
			//std::cout << nn << "  ";


			myMesh::FaceHandle fw = pq.top().second.second;
			ffww.insert(fw.idx());
			//�뾶��Ϊ2
			radius = 1;
			//Ѱ��fw��������Ƭ����������f�ĸ��·�����
			std::vector<myMesh::FaceHandle> fw_neighbor;
			getFaceNeighbor_sharp(mesh, fw, fw_neighbor, true);
			for (int i = 0; i < fw_neighbor.size(); ++i) {
				//���fw���������Ƿ���������������
				if (sharp_face.find(fw_neighbor[i].idx()) == sharp_face.end()) {

					myMesh::Normal nk = mesh.calc_face_normal(fw_neighbor[i]);
					myMesh::Normal nw = mesh.calc_face_normal(fw);
					double length1 = nk.length();
					double length2 = nw.length();
					double theta = 180 / 3.14159*acos(OpenMesh::dot(nk, nw) / (length1*length2));
					double hk = 0;
					if (theta < min_angle) {
						hk = pow((OpenMesh::dot(nk, nw) - cos(min_angle / 180 * 3.14159)*(length1*length2)), 2);
						//std::cout << hk<<"  " << std::endl;
						//std::cout << theta << "  " << std::endl;
					}
					else {
						hk = 0;
					}

					nf += hk* nk;

					//nf = nw;
				}
			}

			//������fw��������֮��Ѽ������nf�Ž�һ��map��
			//std::cout << nf.normalize() <<"  "<< nf.normalize().length() << std::endl;
			if (nf.normalize().length() != 1){
				nf = nn;
				//std::cout << nf.normalize() << "ssssssssss  " << nf.normalize().length() << std::endl;
			}
			//else{
			//	//std::cout << nf.normalize() << "hhhhhhh  " << nf.normalize().length() << std::endl;
			//}
			mesh.set_normal(f, nf.normalize());









			////��������λ��
			//for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f); fv_it.is_valid(); ++fv_it) {
			//	int flag = 0;
			//	double max_move = 0;
			//	for (myMesh::VertexFaceIter vf_it = mesh.vf_begin(*fv_it); vf_it.is_valid(); ++vf_it) {
			//		if (sharp_face.find((*vf_it).idx()) == sharp_face.end()) {
			//			flag = 1;
			//			break;
			//		}
			//	}
			//	//�������������û�з�������
			//	if (flag == 0){
			//		//�Ϳ�ʼ��������
			//		for (int i = 0; i < 20; ++i){
			//			myMesh::VertexHandle vh = *fv_it;



			//			myMesh::Point  ck = mesh.calc_face_centroid(f);//���������ĵ�
			//			myMesh::Normal nk = nf.normalize();
			//			myMesh::Normal nn = nk*OpenMesh::dot(nk, ck - mesh.point(vh));
			//			myMesh::Point npos = mesh.point(vh) + nn;

			//			myMesh::Point a = npos - mesh.point(vh);
			//			double move = OpenMesh::dot(a, a);

			//			//#pragma omp critical
			//			if (move > max_move)
			//				max_move = move;

			//			mesh.set_point(vh, npos);

			//		}

			//	}
			//}










			sharp_face_normal.insert(std::make_pair(f.idx(), nf.normalize()));
			//������ͷԪ��
			pq.pop();

			//����������������ȴ浽v_idx�У��ȴ�һ��һ��ɾ
			v_idx.push_back(f.idx());
			////��������������sharp_face��ɾ��
			//std::map<const int, int>::iterator iter = sharp_face.find(f.idx());
			//sharp_face.erase(iter);

		}
		//����ɾ��һ����
		//mesh.update_face_normals();
		for (int i = 0; i < v_idx.size(); ++i) {
			std::map<const int, int>::iterator iter = sharp_face.find(v_idx[i]);
			sharp_face.erase(iter);
		}
		v_idx.clear();
		max_t++;
	} while (!pq.empty() && max_t<10);
	//��ʱ�����л�ʣ��Ȩֵ����0��û��������Ƭ��������Ƭ������������ȥ������Щ��Ƭ��������Ƭ��ִ�иղŲ���ֱ������Ϊ��
	sharp_face_neiidx.clear();
}


void GELSmooth::getFaceNeighbor_sharp(myMesh & mesh, myMesh::FaceHandle fh, std::vector<myMesh::FaceHandle>& face_neighbor, bool is_include_self)
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
	if (face_neighbor.size() == 0)
		face_neighbor.push_back(fh);
}


myMesh::Normal GELSmooth::GetCalculateNormal(myMesh &mesh, myMesh::FaceHandle fh, double avg_len) {
	if (sharp_face_normal.find(fh.idx()) != sharp_face_normal.end()) {

		return sharp_face_normal[fh.idx()];

	}
	else {
		myMesh::Normal ni = mesh.calc_face_normal(fh);//�����淨��

		return ni;
	}
}
