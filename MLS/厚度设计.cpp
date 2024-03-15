#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件
#include<pcl/search/impl/kdtree.hpp>
#include <pcl/io/ply_io.h>

int main(int argc, char** argv)
{
	//------------------------------------无圆角外壳---------------------------------------------
	double t1 = 19, t2 = 18,t3=18;
	double pc = 2,  c=0, c1 = t1 / 2, c2 = t2 / 2,  c3 = t3 / 2, I1 = t1 * t1 * t1 / 12, I2 = t2 * t2 * t2 / 12, I3 = t3 * t3 * t3 / 12;
	double sigma1 = 0, sigma2 = 0, sigma = 0;
	double h1 = 400, H1 = 250, h2 = 400, H2 = 220, h3 = 250, H3 = 220;
	double alpha1 = H1 / h1, K1 = I2 / I1 * alpha1, alpha2 = H2 / h2, K2 = I3 / I1 * alpha2, alpha3 = H3 / h3, K3 = I3 / I2 * alpha3;
	//短边
	sigma1 = pc * h1 / 2 / t1;
	sigma2 = pc * c1 * h1 * h1 * (1 + alpha1 * alpha1 * K1) / (1 + K1) / 12 / I1;
	sigma = sigma1 + sigma2;
	std::cout << "短边：" << sigma << std::endl;
	//长边
	sigma1 = pc * H1 / 2 / t2;
	sigma2 = pc * c2 * h1 * h1 * (1 + alpha1 * alpha1 * K1) / (1 + K1) / 12 / I2;
	sigma = sigma1 + sigma2;
	std::cout << "长边：" << sigma << std::endl;



	//短边
	sigma1 = pc * h2 / 2 / t1;
	sigma2 = pc * c1 * h2 * h2 * (1 + alpha2 * alpha2 * K2) / (1 + K2) / 12 / I1;
	sigma = sigma1 + sigma2;
	std::cout << "短边：" << sigma << std::endl;
	//长边
	sigma1 = pc * H2 / 2 / t3;
	sigma2 = pc * c3 * h2 * h2 * (1 + alpha2 * alpha2 * K2) / (1 + K2) / 12 / I3;
	sigma = sigma1 + sigma2;
	std::cout << "长边：" << sigma << std::endl;



	
	//短边
	sigma1 = pc * h3 / 2 / t2;
	sigma2 = pc * c2 * h3 * h3 * (1 + alpha3 * alpha3 * K3) / (1 + K3) / 12 / I2;
	sigma = sigma1 + sigma2;
	std::cout << "短边：" << sigma << std::endl;
	//长边
	sigma1 = pc * H3 / 2 / t3;
	sigma2 = pc * c3 * h3 * h3 * (1 + alpha3 * alpha3 * K3) / (1 + K3) / 12 / I3;
	sigma = sigma1 + sigma2;
	std::cout << "长边：" << sigma << std::endl;





	//------------------------------------圆角外壳---------------------------------------------
	std::cout << "圆角容器："<<std::endl;
	double t=25;
	pc = 2, c = t / 2,I1 = t * t * t / 12;
	double r = 30,l=0,L=0;
	double L1 = (400 - 2 * r) / 2, L2 = (400 - 2 * r) / 2, L3 = (250 - 2 * r) / 2, l1 = (250 - 2 * r) / 2, l2 = (220 - 2 * r) / 2, l3 = l2;
	sigma1 = 0, sigma2 = 0, sigma = 0;
	double K = 0;
	double MA = 0;
	double sigma21 = 0, sigma22 = 0;
	//短边
	L = L1;
	l = l1;
	K = -l * l * (3 * r / l * r / l * (2 * L / l - 3.14159 + 2) - 6 * r / l * (1 - L / l) + L / l * L / l * (L / l + 3 + 1.5 * 3.14159 * r / l) - 2) / (3 * (2 * L / l + 3.14159 * r / l + 2));
	MA = pc * K;
	sigma1 = pc * (r + L) / t;
	sigma2 = c / 2 / I1 * (2 * MA + pc * (L * L + 2 * r * L - 2 * r * l));
	sigma = sigma1 + sigma2;
	std::cout << "短边：" << sigma << std::endl;
	//长边
	sigma1 = pc * (r + l) / t;
	sigma21 = MA * c / I1;
	sigma22 = c / 2 / I1 * (2 * MA + pc * L * L);
	if (sigma21 < sigma22) sigma2 = sigma22;
	else sigma2 = sigma21;
	sigma = sigma1 + sigma2;
	std::cout << "长边：" << sigma << std::endl;
	//圆角
	double sita = atan(l / L)*180/ 3.14159;
	sigma1 = pc / t * (sqrt(L * L + l * l) + r);
	sigma2 = c / 2 / I1 * (2 * MA + pc *(2*r*(L*cos(sita)-l*(1-sin(sita)))+L*L));
	std::cout << "sdadasd：" << sita << std::endl;
	std::cout << "sdadasd：" << sigma2 << std::endl;
	sigma = sigma1 + abs(sigma2);
	std::cout << "圆角：" << sigma << std::endl;




	//短边
	L = L2;
	l = l2;
	K = -l * l * (3 * r / l * r / l * (2 * L / l - 3.14159 + 2) - 6 * r / l * (1 - L / l) + L / l * L / l * (L / l + 3 + 1.5 * 3.14159 * r / l) - 2) / (3 * (2 * L / l + 3.14159 * r / l + 2));
	MA = pc * K;
	sigma1 = pc * (r + L) / t;
	sigma2 = c / 2 / I1 * (2 * MA + pc * (L * L + 2 * r * L - 2 * r * l));
	sigma = sigma1 + sigma2;
	std::cout << "短边：" << sigma << std::endl;
	//长边
	sigma1 = pc * (r + l) / t;
	sigma21 = MA * c / I1;
	sigma22 = c / 2 / I1 * (2 * MA + pc * L * L);
	if (sigma21 < sigma22) sigma2 = sigma22;
	else sigma2 = sigma21;
	sigma = sigma1 + sigma2;
	std::cout << "长边：" << sigma << std::endl;
	//圆角
	sita = atan(l / L) * 180 / 3.14159;
	sigma1 = pc / t * (sqrt(L * L + l * l) + r);
	sigma2 = c / 2 / I1 * (2 * MA + pc * (2 * r * (L * cos(sita) - l * (1 - sin(sita))) + L * L));
	std::cout << "sdadasd：" << sita << std::endl;
	std::cout << "sdadasd：" << sigma2 << std::endl;
	sigma = sigma1 + abs(sigma2);
	std::cout << "圆角：" << sigma << std::endl;




	//短边
	L = L3;
	l = l3;
	K = -l * l * (3 * r / l * r / l * (2 * L / l - 3.14159 + 2) - 6 * r / l * (1 - L / l) + L / l * L / l * (L / l + 3 + 1.5 * 3.14159 * r / l) - 2) / (3 * (2 * L / l + 3.14159 * r / l + 2));
	MA = pc * K;
	sigma1 = pc * (r + L) / t;
	sigma2 = c / 2 / I1 * (2 * MA + pc * (L * L + 2 * r * L - 2 * r * l));
	sigma = sigma1 + sigma2;
	std::cout << "短边：" << sigma << std::endl;
	//长边
	sigma1 = pc * (r + l) / t;
	sigma21 = MA * c / I1;
	sigma22 = c / 2 / I1 * (2 * MA + pc * L * L);
	if (sigma21 < sigma22) sigma2 = sigma22;
	else sigma2 = sigma21;
	sigma = sigma1 + sigma2;
	std::cout << "长边：" << sigma << std::endl;
	//圆角
	sita = atan(l / L) * 180 / 3.14159;
	sigma1 = pc / t * (sqrt(L * L + l * l) + r);
	sigma2 = c / 2 / I1 * (2 * MA + pc * (2 * r * (L * cos(sita) - l * (1 - sin(sita))) + L * L));
	std::cout << "sdadasd：" << sita << std::endl;
	std::cout << "sdadasd：" << sigma2 << std::endl;
	sigma = sigma1 + abs(sigma2);
	std::cout << "圆角：" << sigma << std::endl;

	system("pause");
}