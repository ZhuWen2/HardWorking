#include"CloudFusion.h"

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointXYZ point;


int main()
{
    CF cf;
    cf.find_corr();





    //---------------------------------------------读刚才生成的重叠点云-------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    string fileName1 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\工件1配准\\工件1_2_1.ply";
    pcl::io::loadPLYFile(fileName1, *pPntCloud1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    string fileName2 = "C:\\Users\\123456\\Desktop\\测试工件\\线激光\\工件1配准\\工件1_2_2.ply";
    pcl::io::loadPLYFile(fileName2, *pPntCloud2);



    //---------------------------------------------KMeans------------------------------------------------------------
    KMeans km;
    km.SetInputCloud(pPntCloud1, pPntCloud2);

    //-----------------------降采样获得种子点-----------------------
    pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
	pointcloud::Ptr source(new pointcloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pointnormal::Ptr normals(new pointnormal);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;

	//---------------------------去除源点云的NAN点------------------------
	vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*pPntCloud1, *pPntCloud1, indices_src);
	cout << "remove *source_cloud nan" << endl;
	//-------------------------源点云下采样滤波-------------------------

	n.setInputCloud(pPntCloud1);
	n.setNumberOfThreads(8);//设置openMP的线程数
	n.setSearchMethod(tree);
	n.setKSearch(15);
	n.compute(*normals);
	
	// 创建法向空间采样（模板）类对象

	// 设置xyz三个法向空间的分类组数，此处设置为一致，根据具体场景可以调整
	const int kBinNum = 12;
	nss.setBins(kBinNum, kBinNum, kBinNum);
	// 如果传入的是有序点云，此处可以尝试设置为true
	nss.setKeepOrganized(false);
	// 设置随机种子，这样可以保证同样的输入可以得到同样的结果，便于debug分析
	nss.setSeed(0);   // random seed
	// 传入待采样的点云数据
	nss.setInputCloud(pPntCloud1);
	// 传入用于采样分析的法线数据，需与传入点云数据一一对应
	nss.setNormals(normals);
	// 设置采样总数，即目标点云的总数据量
	const float kSampleRatio = 0.05f;
	nss.setSample(pPntCloud1->size() * kSampleRatio);
	// 执行采样并带出采样结果
	nss.filter(*source);
	cout << source->points.size() << endl;
	for (int i = 0; i < (*source).size(); ++i) {
		st_pointxyz p;
		p.x = source->points[i].x;
		p.y = source->points[i].y;
		p.z = source->points[i].z;
		km.pc_arr.push_back(p);

	}
    //km.SetInitKCenter(pPntCloud1, pPntCloud2,nss);
	clock_t start = clock();
	km.SetK(km.pc_arr.size());
	km.InitKCenter(km.pc_arr);
	km.Cluster();
	//km.SaveFile("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\工件1配准\\kmeans","工件1");
	km.Mix("C:\\Users\\123456\\Desktop\\测试工件\\线激光\\工件1配准\\kmeans", "工件1");
	clock_t end = clock();
	std::cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
    //cout << km.pc_arr.size() << endl;
    //std::cout << "time = " << double(cf.whole_time) / CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
   // cout << "h" << endl;
    system("pause");

    return (0);
}