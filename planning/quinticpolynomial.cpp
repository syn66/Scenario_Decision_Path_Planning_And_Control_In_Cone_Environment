
#include "quinticpolynomial.h"

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

	// y对x
	Eigen::Matrix3d C;
	C << std::pow(xe_, 3), std::pow(xe_, 4), std::pow(xe_, 5),
        3 * std::pow(xe_, 2), 4 * std::pow(xe_, 3),
        5 * std::pow(xe_, 4), 6 * xe_, 12 * std::pow(xe_, 2),
        20 * std::pow(xe_, 3);
	Eigen::Vector3d D;
	D << ye - b0 - b1 * xe_ - b2 * std::pow(xe_, 2),
        vye - b1 - 2 * b2 * xe_, aye - 2 * b2;
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
double QuinticPolynomial::compute_X(double t) {
	return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
		a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
};

/**
 * 输入： 时间t
 * 输出： dot_x(t)
 * 作用： 求 时刻t 对x的 一阶导数
**/
double QuinticPolynomial::compute_X_d(double t) {
	return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) +
		a5 * std::pow(t, 4);
};

/**
 * 输入： 时间t
 * 输出： dot_dot_x(t)
 * 作用： 求 时刻t 对x的 二阶导数
**/
double QuinticPolynomial::compute_X_dd(double t) {
	return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) +
		20 * a5 * std::pow(t, 3);
};

/**
 * 输入： 时间t
 * 输出： dot_dot_dot_x(t)
 * 作用： 求 时刻t 对x的 三阶导数
**/
double QuinticPolynomial::compute_X_ddd(double t) {
	return 6 * a3 + 24 * a4 * t + 60 * a5 * std::pow(t, 2);
};

/**
 * 输入： x坐标
 * 输出： y坐标
 * 作用： 求 x 下的 y
**/
double QuinticPolynomial::compute_Y(double x) {
	return b0 + b1 * x + b2 * std::pow(x,2) + b3 * std::pow(x,3) + 
			b4 * std::pow(x,4) + b5 * std::pow(x,5);
};

/**
 * 输入： x坐标
 * 输出： dot_y(x)
 * 作用： 求 一阶导数
**/
double QuinticPolynomial::compute_Y_xd(double x) {
	return b1 + 2 * b2 * x + 3 * b3 * std::pow(x, 2) + 4 * a4 * std::pow(x, 3) +
           5 * b5 * std::pow(x, 4);
};

/**
 * 输入： x坐标
 * 输出： dot_dot_y(x)
 * 作用： 求二阶导数
**/
double QuinticPolynomial::compute_Y_xdd(double x) {
    return 2 * b2 + 6 * b3 * x + 12 * b4 * std::pow(x, 2) +
           20 * b5 * std::pow(x, 3);
};

/**
 * 输入： dot_y(x), dot_x(t)
 * 输出： dot_y(t)
 * 作用： 求y对t的一阶导数
**/
double QuinticPolynomial::compute_Y_td(double x, double t) {
	double dot_x = compute_X_d(t);
	double x_t = compute_X(t);
	return b1 * dot_x + 2 * b2 * dot_x * x_t + 3 * b3 * dot_x * std::pow(x_t, 2) + 
			4 * b4 * dot_x * std::pow(x_t,3) + 5 * b5 * dot_x * std::pow(x_t,4);
};

/**
 * 输入： dot_dot_y(x), dot_x(t), dot_dot_x(t)
 * 输出： dot_dot_y(x)
 * 作用： 求y对t的二阶导数
**/
double QuinticPolynomial::compute_Y_tdd(double x, double t) {
	double dot_dot_yt = compute_X_dd(t) * std::pow(compute_X_d(t), 2) + compute_Y_xd(x) * compute_X_dd(t);
	return dot_dot_yt;
};


double QuinticPolynomial::compute_theta(double x, double t) {
	double theta = std::atan2(compute_Y_td(x,t) ,compute_X_d(t));
	if ( theta > M_PI) {
        theta -= 2*M_PI;
    }
    else if ( theta < -M_PI) {
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
		double theta_temp;
		double cur_temp;
		Path path_temp;

		xy_temp.m_X = compute_X(t);
		xy_temp.m_Y = compute_Y(xy_temp.m_X);
		
		// 夹角    !!!!!!!!!!!
		// theta_temp = -std::atan2(compute_Y_td(xy_temp.m_X,t) ,compute_X_d(t));
		// double dy = compute_Y(compute_X(t+0.1)) - compute_Y(compute_X(t));
		// double dx = compute_X(t+0.1) - compute_X(t);
		theta_temp = compute_theta(xy_temp.m_X, t);
		//曲率
		cur_temp = compute_Y_xdd(xy_temp.m_X) / std::pow((1 + std::pow(compute_Y_xd(xy_temp.m_X), 2)), 1.5);

		path_temp.xy = xy_temp;
		path_temp.theta_des = theta_temp;
		path_temp.cur = cur_temp;

		path.push_back(path_temp);
	}

	return path;
};
