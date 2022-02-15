#pragma once

#include "eigen3/Eigen/Dense"
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "../config.h"
#include "../utils.h"


class QuinticPolynomial {
private:
    // cheliangdeweizhi起点 坐标， 速度 加速度 zongxiang
    double xs;
    double vxs;
    double axs;

    //  zongxiang坐标， 速度 加速度
    double xe;
    double vxe;
    double axe;

    // cheliangdeweizhi起点 坐标， 速度 加速度 hengxiang
    double ys;
    double vys;
    double ays;

    //zhongdian  横向坐标， 速度 加速度
    double ye;
    double vye;
    double aye;


    //五次多项式系数
    double a0, a1, a2, a3, a4, a5;  // x(t)
    double b0, b1, b2, b3, b4, b5;   // y(x)

    double LC_T; // 换道时间

    bool collisionCheck(std::vector<XY>& cone_XY);

    double compute_X(double t);

    double compute_X_d(double t);

    double compute_X_dd(double t);

    double compute_X_ddd(double t);

    double compute_Y(double x);

    double compute_Y_xd(double x);

    double compute_Y_xdd(double x);

    double compute_Y_td(double x, double t);

    double compute_Y_tdd(double x, double t);

    double compute_theta(double x,  double t);

public:
    QuinticPolynomial(){};

    //五次多项式系数
    QuinticPolynomial(double xs_, double vxs_, double axs_,
                            double xe_, double vxe_, double axe_,
                            double ys_, double vys_, double ays_,
                            double ye_, double vye_, double aye_,
                            double T);

    std::vector<Path> quinticPlan();
};
