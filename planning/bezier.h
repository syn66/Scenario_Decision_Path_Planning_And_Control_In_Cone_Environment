#pragma once

#include <iostream>
#include <vector>

#include "../config.h"
#include "../utils.h"

class Bezier
{
private:
    std::vector<XY> way_points; //所有路径点
    std::vector< std::vector<XY> > control_Points; // 控制点
    size_t main_order; //主体贝塞尔曲线阶数
    size_t number_of_line; //有几段曲线
    MergeType Smooth_merge_type; // 最后两段贝塞尔曲线平滑拼接类型


    void Judge_Order_and_SmoothType(); //判断主体贝塞尔曲线阶数，并判断最后两条贝塞尔曲线拼接类型

    void divide_way_points();

    // double compute_curvature(size_t order, double t, std::vector<XY>& points);

    XY compute_aide_point(XY P3, XY P4, XY P5, XY Q2, MergeType T); //计算辅助控制点

    std::vector<Path> compute_Quintic(XY& point0, XY& point1, XY& point2,
                            XY& point3, XY& point4, XY& point5);

    std::vector<Path> compute_Cubic(XY& point0, XY& point1, XY& point2, XY& point3);

    double compute_theta();



public:
    Bezier(const std::vector<XY>& rough_path);

    std::vector<Path> Smooth();

    void showpoints();
};