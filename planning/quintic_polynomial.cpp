
#include "quintic_polynomial.h"

/**
 * 输入： 起点， 终点， 换道时间
 * 输出： 无
 * 作用： 构造函数 求解五次多项式系数
**/
QuinticPolynomial::QuinticPolynomial(   double xs_, double vxs_, double axs_,
													double xe_, double vxe_, double axe_, 
													double ys_, double vys_, double ays_,
													double ye_, double vye_, double aye_,
													double T):
															xs(xs_), vxs(vxs_), axs(axs_),
															xe(xe_), vxe(vxe_), axe(axe_),
															a0(xs_), a1(vxs_), a2(axs_ / 2.0),
															ys(ys_), vys(vys_), ays(ays_),
															ye(ye_), vye(vye_), aye(aye_),
															b0(ys_), b1(vys_), b2(ays_ / 2.0),
															LC_T(T) {
	// x对t
	Eigen::Matrix3f A;
	A << std::pow(this->LC_T, 3), std::pow(this->LC_T, 4), std::pow(this->LC_T, 5), 3 * std::pow(this->LC_T, 2),
		4 * std::pow(this->LC_T, 3), 5 * std::pow(this->LC_T, 4), 6 * this->LC_T, 12 * std::pow(this->LC_T, 2),
		20 * std::pow(this->LC_T, 3);
	Eigen::Vector3f B;
	B << xe - a0 - a1 * this->LC_T - a2 * std::pow(this->LC_T, 2), vxe - a1 - 2 * a2 * this->LC_T,
		axe - 2 * a2;

	Eigen::Vector3f c_eigen = A.colPivHouseholderQr().solve(B);
	this->a3 = c_eigen[0];
	this->a4 = c_eigen[1];
	this->a5 = c_eigen[2];

	// y对t
	Eigen::Matrix3d C;
	C << std::pow(this->LC_T, 3), std::pow(this->LC_T, 4), std::pow(this->LC_T, 5), 3 * std::pow(this->LC_T, 2),
		4 * std::pow(this->LC_T, 3), 5 * std::pow(this->LC_T, 4), 6 * this->LC_T, 12 * std::pow(this->LC_T, 2),
		20 * std::pow(this->LC_T, 3);
	Eigen::Vector3d D;
	D << ye - b0 - b1 * this->LC_T - b2 * std::pow(this->LC_T, 2),
        vye - b1 - 2 * b2 * this->LC_T , aye - 2 * b2;
	Eigen::Vector3d b_eigen = C.colPivHouseholderQr().solve(D);

	this->b3 = b_eigen[0];
	this->b4 = b_eigen[1];
	this->b5 = b_eigen[2];
};

/**
 * 输入： 时间t
 * 输出：  x(t)
 * 作用：  求 t 时刻的 x
**/
double QuinticPolynomial::compute_Xt(double t) {
	return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
		a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
};

/**
 * 输入： 时间t
 * 输出： dot_x(t)
 * 作用： 求 时刻t 对x的 一阶导数
**/
double QuinticPolynomial::compute_dXt(double t) {
	return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) +
		a5 * std::pow(t, 4);
};

/**
 * 输入： 时间t
 * 输出： dot_dot_x(t)
 * 作用： 求 时刻t 对x的 二阶导数
**/
double QuinticPolynomial::compute_ddXt(double t) {
	return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) +
		20 * a5 * std::pow(t, 3);
};

/**
 * 输入： t
 * 输出： y坐标
 * 作用： 求 x 下的 y
**/
double QuinticPolynomial::compute_Yt(double t) {
	return b0 + b1 * t + b2 * std::pow(t,2) + b3 * std::pow(t,3) + 
			b4 * std::pow(t,4) + b5 * std::pow(t,5);
};

/**
 * 输入： t
 * 输出： dot_y(x)
 * 作用： 求 一阶导数
**/
double QuinticPolynomial::compute_dYt(double t) {
	return b1 + 2 * b2 * t + 3 * b3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) +
           5 * b5 * std::pow(t, 4);
};

/**
 * 输入： t
 * 输出： dot_dot_y(x)
 * 作用： 求二阶导数
**/
double QuinticPolynomial::compute_ddYt(double t) {
    return 2 * b2 + 6 * b3 * t + 12 * b4 * std::pow(t, 2) +
           20 * b5 * std::pow(t, 3);
};

/**
 * 输入： 
 * 输出： 
 * 作用：
**/
double QuinticPolynomial::compute_curvature(double t) {
	double curvature = fabs( compute_dYt(t) * compute_ddXt(t) - compute_dXt(t) * compute_ddYt(t)) / 
						std::pow((std::pow(compute_dXt(t),2) + std::pow(compute_dYt(t),2)), 1.5);
	return curvature;
};


double QuinticPolynomial::compute_theta(double t) {
	double vx = this->compute_Xt(t);
	double vy = this->compute_Yt(t);
	double theta = std::atan2(vy, vx);

	// if ( theta > M_PI) {
    //     theta -= 2*M_PI;
    // }
    // else if ( theta < -M_PI) {
    //     theta += 2*M_PI;
    // }

	if (theta < 0) {
		theta += 2*M_PI;
	}
	return theta;
};

/**
 * 输入： 无
 * 输出： 五次多项式轨迹
 * 作用： 接口-五次多项式规划
**/
std::vector<Path> QuinticPolynomial::quinticPlan() {

	std::vector<Path> path;
	for (double t = 0; t < this->LC_T; t+=0.1) {
		XY xy_temp;
		double yaw_temp;
		double cur_temp;
		Path path_temp;
		double vx_temp;

		xy_temp.m_X = this->compute_Xt(t);
		xy_temp.m_Y = this->compute_Yt(t);

		yaw_temp = this->compute_theta(t);
		//曲率
		cur_temp = this->compute_curvature(t);

		path_temp.xy = xy_temp;
		path_temp.yaw_des = yaw_temp;
		path_temp.cur = cur_temp;

		path.push_back(path_temp);
	}

	return path;
};
