#include "MeshSmooth.h"
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <minmax.h>
#include <algorithm>
#include <math.h>

typedef OpenMesh::VectorT<float, 3> Vector3;



void GELSmooth::Sharp(myMesh &mesh, int max_iter, int &count_s, std::vector<std::vector<const int> >& sharp_face_neiidx)
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
		avg_len += mesh.calc_edge_length(myMesh::EdgeHandle(e_it));
	}

	std::vector<myMesh::Normal> filter_normals;
	filter_normals.resize(mesh.n_faces(), myMesh::Normal(0.0));
	for (int iter = 0; iter < max_iter; ++iter)//迭代max_iter次
	{
		int f_it = 0, f_size = mesh.n_faces();//一共有几个面
		//#pragma omp parallel for
		//先遍历一遍所有面片，找到特征面片并保存到map里，同时把各自的邻域面片保存到二维数组里
		for (f_it = 0; f_it < f_size; ++f_it)//循环f_size次
		{
			SearchFeatures(mesh, myMesh::FaceHandle(f_it), count_s, sharp_face_neiidx);

		}
		//这个函数的功能是遍历刚才找到的特征面片，计算权值，把计算得到的面片索引和法向量放到一个新的map里
		CalculateNormal(mesh, myMesh::FaceHandle(f_it), sharp_face_neiidx);
		for (f_it = 0; f_it < f_size; ++f_it)//循环f_size次
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
		//下面调整点位置
		//用刘世凡博士论文的双边滤波顶点调整方法。
				std::vector<myMesh::Point> vertex_positions(mesh.n_vertices(), myMesh::Point(0.0));
				std::vector<int> count(mesh.n_vertices(), 0);
				for (int sub_iter = 0; sub_iter < 10; ++sub_iter)//为什么是10次循环
				{
					//每次遍历所有半边，根据每个半边的面法线和此半边的关系调整此半边的出发顶点，
					//并记录每个顶点操作了几次
					//下面的for循环遍历每个半边，每个面里有三个半边，最后每个顶点被操作的次数就是顶点相邻的面的个数
					for (myMesh::HalfedgeIter he_it = mesh.halfedges_begin(); he_it != mesh.halfedges_end(); ++he_it)
					{
						//获取he_it半边所在的面fh
						myMesh::FaceHandle fh = mesh.face_handle(*he_it);
						if (fh.is_valid())
						{
							myMesh::VertexHandle vh = mesh.from_vertex_handle(*he_it);
							//std::cout << vh << std::endl;
							//获得上一个半边
							myMesh::HalfedgeHandle prev_heit = mesh.prev_halfedge_handle(*he_it);
							myMesh::Normal dir2 = mesh.point(mesh.to_vertex_handle(prev_heit)) - mesh.point(vh);
							myMesh::Normal dir = mesh.point(mesh.to_vertex_handle(*he_it)) - mesh.point(vh);
		
							myMesh::Normal n = filter_normals[fh.idx()];
							myMesh::Normal Vi = n * (OpenMesh::dot(n, dir) + OpenMesh::dot(n, dir2));
							//vertex_positions[vh.idx()] += mesh.point(vh) + 0.5 * n * OpenMesh::dot(n, dir);
		
							myMesh::Point  ck = mesh.calc_face_centroid(fh);//计算面中心点
							myMesh::Normal nk = filter_normals[fh.idx()];
							myMesh::Normal nn = nk*OpenMesh::dot(nk, ck - mesh.point(vh));
							vertex_positions[vh.idx()] += mesh.point(vh) + nn;
							//vertex_positions[vh.idx()] += mesh.point(vh) + 0.33 * Vi;
							count[vh.idx()] += 1;//记录每个顶点操作了几次，最后应该次数是顶点相邻的面的个数。
		
		
		
							//myMesh::VertexHandle vh = mesh.from_vertex_handle(*he_it);
							//myMesh::Normal dir = mesh.point(mesh.to_vertex_handle(*he_it)) - mesh.point(vh);
							//myMesh::Normal n = filter_normals[fh.idx()];
							//vertex_positions[vh.idx()] += mesh.point(vh) + 0.5 * n * OpenMesh::dot(n, dir);
							//count[vh.idx()] += 1;//记录每个顶点操作了几次，最后应该次数是顶点相邻的面的个数。
		
						}
					}
		
					double max_move = 0;
					int v_it = 0, v_size = mesh.n_vertices();//顶点的个数
		//#pragma omp parallel for//下面的循环把上面算出来的vertex_positions除以count[v_it]得到平均的位置npos，
					//并调整到npos位置上。
					for (v_it = 0; v_it < v_size; ++v_it)//循环v_size次
					{
						myMesh::Point npos;
						int j = v_it;
						if (count[j] == 0)//为什么会有顶点没被操作过？
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
						mesh.set_point(myMesh::VertexHandle(v_it), npos);//把v_it所在顶点调整到npos的位置上去
					}
					//如果最大的移动距离都已经小于阈值，就不用再调整了，直接break推出循环。
					if (max_move < (1e-8*avg_len)*(1e-8*avg_len))
					{
						break;
					}
				}
		filter_normals.clear();
		mesh.update_face_normals();//计算面法线信息
	}
}

//寻找特征
void GELSmooth::SearchFeatures(myMesh & mesh, myMesh::FaceHandle fh, int &count_s, std::vector<std::vector<const int> >& sharp_face_neiidx)
{

	//此函数是把索引为fh的这个面的邻接面找出来然后遍历邻接面并利用这些邻接面的信息代入到双边滤波公式中，最后得到双边滤波完的此面的法向量。



	//创建此面的法向量filter_normal
	myMesh::Normal filter_normal = myMesh::Normal(0.0, 0.0, 0.0);
	//创建此面邻域面的法向量容器face_neighbor
	std::vector<myMesh::FaceHandle> face_neighbor;
	//计算邻域面索引放入face_neighbor中
	getFaceNeighbor_sharp(mesh, fh, face_neighbor, true);
	myMesh::Normal ni = mesh.calc_face_normal(fh);//计算面法线


	int count1 = 0;

	//建立一个容器保存邻域面的法向量，等待后面计算它们两两之间的夹角，取夹角最小的做判断。
	std::vector<myMesh::Normal> nei_normal;
	std::vector<const int> nei_idx;
	nei_normal.push_back(ni);
	nei_idx.push_back(fh.idx());
	//遍历这些邻面

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


	//划分索引保存到集合里

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




//这个函数先遍历各个特征面，计算权重，在外围的特征面会先被调整法向量，然后将这个面片的顶点调整。但需要注意的是如果按照论文的方法调整，三个顶点都会
//被调整，导致塌陷现象。所以自己加了一个改动，就是与非特征面相邻的顶点不会被调整，这样就避免了塌陷的问题。但与此同时，也不能每处理完一个特征面片就
//改成非特征面，因为这样会影响到旁边的面片，那样旁边的面片三个顶点都会与非特征面相邻，顶点就不被会调整了。所以要把优先队列中的权重不为零的，即
//与非特征面相邻的所有面片全部调整完之后再设置为非特征面。
//所以有两点与论文不同
//1.与非特征面相邻的顶点不调整位置
//2.处理完的特征面设置为非特征面的时机不同。
void GELSmooth::CalculateNormal(myMesh & mesh, myMesh::FaceHandle fh, std::vector<std::vector<const int> >& sharp_face_neiidx)
{
	int max_t = 0;
	do {
		//std::cout << pq.size() << std::endl;
		while (!pq.empty()) pq.pop();
		//遍历特征面片
		for (auto it_fe = sharp_face.cbegin(); it_fe != sharp_face.cend(); ++it_fe) {
			//最大权值
			double max_w = 0;

			//相似面片fw先设为本身
			myMesh::FaceHandle fw = myMesh::FaceHandle((*it_fe).first);
			double w = 0;
			int neiidx = (*it_fe).second;
			//遍历这个特征面片的邻域面
			for (int i = 0; i < sharp_face_neiidx[neiidx].size(); ++i) {
				//每个邻域面idx设为 n
				const int n = sharp_face_neiidx[neiidx][i];

				//如果邻域面属于非特征面
				if (sharp_face.find(n) == sharp_face.end()) {
					myMesh::Normal ni = mesh.calc_face_normal(myMesh::FaceHandle((*it_fe).first));//计算面法线
					myMesh::Normal nj = mesh.calc_face_normal(myMesh::FaceHandle(n));//计算面法线
					w = OpenMesh::dot(ni, nj);
					if (w > max_w) {
						max_w = w;

						fw = myMesh::FaceHandle(n);

					}

					if (w < 0) w = 0;


				}

			}
			//std::cout <<  std::endl;
			//遍历完邻域面之后，把权值和索引加入到优先队列
			std::pair<myMesh::FaceHandle, myMesh::FaceHandle> p = std::make_pair(myMesh::FaceHandle((*it_fe).first), fw);
			pq.push(std::make_pair(w, p));


		}
		std::vector<const int> v_idx;
		//然后依次出队计算更新的法向量
		while (pq.top().first){
			//定义更新后的法向量nf
			myMesh::Normal nf = myMesh::Normal(0.0, 0.0, 0.0);
			//特征面片f和它的相似面片fw
			myMesh::FaceHandle f = pq.top().second.first;

			myMesh::Normal nn = mesh.calc_face_normal(f);
			//std::cout << nn << "  ";


			myMesh::FaceHandle fw = pq.top().second.second;
			ffww.insert(fw.idx());
			//半径设为2
			radius = 1;
			//寻找fw的相邻面片并用来计算f的更新法向量
			std::vector<myMesh::FaceHandle> fw_neighbor;
			getFaceNeighbor_sharp(mesh, fw, fw_neighbor, true);
			for (int i = 0; i < fw_neighbor.size(); ++i) {
				//如果fw的相邻面是非特征面则参与计算
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

			//遍历完fw的邻域面之后把计算完的nf放进一个map里
			//std::cout << nf.normalize() <<"  "<< nf.normalize().length() << std::endl;
			if (nf.normalize().length() != 1){
				nf = nn;
				//std::cout << nf.normalize() << "ssssssssss  " << nf.normalize().length() << std::endl;
			}
			//else{
			//	//std::cout << nf.normalize() << "hhhhhhh  " << nf.normalize().length() << std::endl;
			//}
			mesh.set_normal(f, nf.normalize());









			////调整顶点位置
			//for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(f); fv_it.is_valid(); ++fv_it) {
			//	int flag = 0;
			//	double max_move = 0;
			//	for (myMesh::VertexFaceIter vf_it = mesh.vf_begin(*fv_it); vf_it.is_valid(); ++vf_it) {
			//		if (sharp_face.find((*vf_it).idx()) == sharp_face.end()) {
			//			flag = 1;
			//			break;
			//		}
			//	}
			//	//遍历完邻面如果没有非特征面
			//	if (flag == 0){
			//		//就开始调整顶点
			//		for (int i = 0; i < 20; ++i){
			//			myMesh::VertexHandle vh = *fv_it;



			//			myMesh::Point  ck = mesh.calc_face_centroid(f);//计算面中心点
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
			//弹出队头元素
			pq.pop();

			//把这个特征面索引先存到v_idx中，等待一会一起删
			v_idx.push_back(f.idx());
			////并把这个特征面从sharp_face中删掉
			//std::map<const int, int>::iterator iter = sharp_face.find(f.idx());
			//sharp_face.erase(iter);

		}
		//现在删除一部分
		//mesh.update_face_normals();
		for (int i = 0; i < v_idx.size(); ++i) {
			std::map<const int, int>::iterator iter = sharp_face.find(v_idx[i]);
			sharp_face.erase(iter);
		}
		v_idx.clear();
		max_t++;
	} while (!pq.empty() && max_t<10);
	//此时队列中还剩下权值都是0的没有相似面片的特征面片，接下来返回去再算这些面片的相似面片并执行刚才操作直到队列为空
	sharp_face_neiidx.clear();
}


void GELSmooth::getFaceNeighbor_sharp(myMesh & mesh, myMesh::FaceHandle fh, std::vector<myMesh::FaceHandle>& face_neighbor, bool is_include_self)
{
	face_neighbor.clear();
	if (is_include_self)//如果传进来的是true，那么就包括传进来的面mesh本身，把其索引fh放进容器。
	{
		face_neighbor.push_back(fh);
	}

	//计算面fh的一共几圈面片

	//接下来用循环器循环每个面的顶点
	for (myMesh::FaceVertexIter fv_it = mesh.fv_begin(fh); fv_it.is_valid(); ++fv_it)
	{
		//下面这个循环是用循环器循环每个顶点邻域的面
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
			}//如果不是传进来的fh这个面，就把它push到face_neighbor中。//这里暂时没考虑重复的问题，因为下面会去除重复，下面的去重方法听师兄说会比直接存放在set中快。

		}
	}
	//std::unique函数的作用是“去除”容器或者数组中相邻元素的重复出现的元素，注意 
	//(1) 这里的去除并非真正意义的erase，而是将重复的元素放到容器的末尾，返回值是去重之后的尾地址。
	//(2) unique针对的是相邻元素，所以对于顺序顺序错乱的数组成员，或者容器成员，需要先进行排序，可以调用std::sort()函数
	std::sort(face_neighbor.begin(), face_neighbor.end());//升序排序
	face_neighbor.erase(std::unique(face_neighbor.begin(), face_neighbor.end()), face_neighbor.end());
	//所以这个意思是把std::unique返回的尾地址直到face_neighbor.end()给删除掉。
	if (face_neighbor.size() == 0)
		face_neighbor.push_back(fh);
}


myMesh::Normal GELSmooth::GetCalculateNormal(myMesh &mesh, myMesh::FaceHandle fh, double avg_len) {
	if (sharp_face_normal.find(fh.idx()) != sharp_face_normal.end()) {

		return sharp_face_normal[fh.idx()];

	}
	else {
		myMesh::Normal ni = mesh.calc_face_normal(fh);//计算面法线

		return ni;
	}
}
