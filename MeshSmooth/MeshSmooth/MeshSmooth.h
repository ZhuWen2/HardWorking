#pragma once 
#define _USE_MATH_DEFINES 1
#define OM_STATIC_BUILD 1
#include <queue>
#include <unordered_set>
#include <set>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <map>

extern int radius;
extern int smooth_area;

struct myMeshTraits : public OpenMesh::DefaultTraits
{
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;
};

typedef OpenMesh::TriMesh_ArrayKernelT<myMeshTraits> myMesh;

//extern std::vector<myMesh::FaceHandle> faceh;
//extern  std::unordered_set<int> face_arris;
//extern  std::unordered_set<int> face_plane;
extern std::set<const int> face_arris1;
extern std::set<const int> face_arris2;
extern std::set<const int> face_arris3;
extern std::map<const int, int> sharp_face;
//计算权值的优先队列
extern std::priority_queue<std::pair<double, std::pair<myMesh::FaceHandle, myMesh::FaceHandle>>, std::vector<std::pair<double, std::pair<myMesh::FaceHandle, myMesh::FaceHandle>>>, std::less<std::pair<double, std::pair<myMesh::FaceHandle, myMesh::FaceHandle>>> > pq;
//放计算完特征面片的索引和法向量的map
extern std::map<const int, myMesh::Normal> sharp_face_normal;
//特征面片
extern std::set<const int> ffww;
extern double min_angle;
extern int model;
extern int max_sharp_num;
extern int sharp_num;

struct HalfEdgeCounter {
	int touched;
	bool isRemovedFromQueue;
};

struct  PQElement
{
	double pri;
	myMesh::HalfedgeHandle h;
	int time;

	//PQElement() {}
	PQElement(double _pri, myMesh::HalfedgeHandle _h, int _time) :
		pri(_pri), h(_h), time(_time) {}
};

inline bool operator<(const PQElement & e0, const PQElement & e1)
{
	return e0.pri > e1.pri;
}

class  EnergyFun
{
public:
	virtual double delta_energy(const myMesh& m, myMesh::HalfedgeHandle h) const = 0;
	virtual double energy(const myMesh& m, myMesh::HalfedgeHandle h) const { return 0; }
};

class  DihedralEnergy : public EnergyFun
{
public:

	DihedralEnergy(double _gamma = 4.0, bool _use_alpha = false) :
		gamma(_gamma), use_alpha(_use_alpha) {}
	~DihedralEnergy() {}

	double energy(const myMesh& m, myMesh::HalfedgeHandle h) const;

	double delta_energy(const myMesh& m, myMesh::HalfedgeHandle h) const;

	inline double min_angle(const myMesh& m, myMesh::HalfedgeHandle h) const
	{
		compute_angles(m, h);
		return (std::min)((std::min)((std::min)((std::min)(aa_12, aa_b1), aa_c1), aa_2a), aa_2d);
	}

private:
	const double gamma;
	const bool use_alpha;

	inline double cos_ang(const OpenMesh::Vec3d& n1, const OpenMesh::Vec3d& n2) const
	{
		return (std::max)(-1.0, (std::min)(1.0, OpenMesh::dot(n1, n2)));
	}

	inline double edge_alpha_energy(OpenMesh::Vec3d v1, OpenMesh::Vec3d v2, double ca) const
	{
		return pow((v1 - v2).length() *(acos(ca)), 1.0f / gamma);
	}

	inline double edge_c_energy(OpenMesh::Vec3d v1, OpenMesh::Vec3d v2, double ca) const
	{
		return pow((v1 - v2).length()*(1 - ca), 1.0f / gamma);
	}

	void compute_angles(const myMesh& m, myMesh::HalfedgeHandle h) const;

	mutable double ab_12;
	mutable double ab_a1;
	mutable double ab_b1;
	mutable double ab_2c;
	mutable double ab_2d;

	mutable double aa_12;
	mutable double aa_b1;
	mutable double aa_c1;
	mutable double aa_2a;
	mutable double aa_2d;
};


class GELSmooth
{
public:
	GELSmooth() {};
	~GELSmooth() {};

	void LaplaceSmooth(myMesh &mesh, int max_iter);
	void denoise(myMesh &mesh, int max_iter);
	void MLSSmooth(myMesh &mesh, int max_iter);
	void optimize_mesh(myMesh& m);
	void Sharp(myMesh &mesh, int max_iter, int &count_s, std::vector<std::vector<const int> >& sharp_face_neiidx);
	void SearchFeatures(myMesh & mesh, myMesh::FaceHandle fh, int &count_s, std::vector<std::vector<const int> >& sharp_face_neiidx);
	void CalculateNormal(myMesh & mesh, myMesh::FaceHandle fh, std::vector<std::vector<const int> >& sharp_face_neiidx);
protected:
	//双边滤波得到滤波后的面法向量
	myMesh::Normal GELSmoothFilter(myMesh &mesh, myMesh::FaceHandle fh, double avg_len);
	myMesh::Normal GELSmoothFilter_pro(myMesh &mesh, myMesh::FaceHandle fh, double avg_len);
	void GELSmoothFilter_MLSpro(myMesh &mesh, myMesh::FaceHandle fh, double avg_len);
	myMesh::Normal GetCalculateNormal(myMesh &mesh, myMesh::FaceHandle fh, double avg_len);
	//获取一个面的所有领域面的索引FaceHandle
	void getFaceNeighbor(myMesh & mesh, myMesh::FaceHandle fh, std::vector<myMesh::FaceHandle>& face_neighbor, bool is_include_self);
	void getFaceNeighbor_sharp(myMesh & mesh, myMesh::FaceHandle fh, std::vector<myMesh::FaceHandle>& face_neighbor, bool is_include_self);

private:
	void priority_queue_optimization(myMesh& m, const EnergyFun& efun);
	void add_to_queue(const myMesh& m, std::vector<HalfEdgeCounter>& counter, std::priority_queue<PQElement>& Q, myMesh::HalfedgeHandle h, const EnergyFun& efun, std::vector<int> & flipCounter, int time);
	void add_one_ring_to_queue(const myMesh& m, std::vector<HalfEdgeCounter>& touched, std::priority_queue<PQElement>& Q, myMesh::VertexHandle v, const EnergyFun& efun, std::vector<int> & flipCounter, int time);
};
