#ifndef _DELAUNAY_
#define _DELAUNAY_

#include <vector>
#include <list>
#include <utility>

#include "eigen3/Eigen/Dense"

#include "../config.h"
#include "../utils.h"
// #include "smoother.h"
#include "bezier.h"


class Delaunay_Planner
{
private:

    static double Max_angle_change_cost;   // 车辆最大转角
    static double Cone_gap_cost;   //   锥桶间隔是大致相等的
    static double Smooth_path_cost;   // 路径大致 是平滑的无较大突变
    // static double Sqrt_between_PathandSensor_cost;  //路径的长度和传感器探测范围的平方差 ，用来惩罚过长和过短的路径
    static double Cone_gap;  // 锥桶摆放间隔 ，数据来源 校门口

    std::vector< std::pair<Point, unsigned> > points;  // 传入的锥桶的坐标 德劳内Point类型

    std::vector<DPoint> Delaunay_Middle_point;  // 德劳内的边的 中点
    std::vector<Edge> edge_vec; // 德劳内三角的边

    void Init( std::vector<XY> cone_xy );

    void Discrete_space_Find_middle_point();

    int calc_cost_G(DPoint* parent_point, DPoint* child_point);
    int calc_cost_A(DPoint* parent_point, DPoint* child_point);
    int calc_cost_D(DPoint* parent_point, DPoint* child_point);

    DPoint* calc_point_cost(DPoint* current_Point);

public:
    
    Delaunay_Planner() = default;

    Delaunay_Planner(const std::vector<XY>& cone_vec);

    std::vector<Path> Plan();

    std::vector< std::pair<Point, unsigned> >& getPoints();

    std::vector<Edge>&  getEdgevec();

    std::vector<DPoint>& getMidPoints();

};
#endif /*_DELAUNAY_*/