#ifndef _CONFIG_
#define _CONFIG_

#include "eigen3/Eigen/Dense"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

// CGAL 库
typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                       Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                    Delaunay;
typedef Kernel::Point_2                                                Point;


// 常量 车道宽 m
const double LaneWidth = 3.75; //m
const double car_length = 4.4; //m
const double CarWidth = 2.0;  // m
const double ConeErro = 0.4;  //m
const int DetectRange = 30;  //m 采样范围
const double LaneChangeRange = 20; // m
const int mapUpdatestep = 0.05; //m 5cm
const double maxAccl = 5; // m/s2
const double maxspeed = 10; //m/s 
const double ts = 0.01; //时间间隔

// 锥桶坐标 结构体  坐标在 车辆坐标系下
struct XY
{
    double m_X;
    double m_Y;

    bool operator!=(XY& xy) {
        bool equal = false;
        if (this->m_X == xy.m_X && this->m_Y == xy.m_Y ) {
            equal = true;
        }
        return equal;
    };

    XY& operator-(XY& xy) {
        static XY xy_temp;
        xy_temp.m_X = this->m_X - xy.m_X;
        xy_temp.m_Y = this->m_Y - xy.m_Y;
        return xy_temp;
    };

    XY& operator*(double n) {
        static XY xy_temp;
        xy_temp.m_X = n * this->m_X;
        xy_temp.m_Y = n * this->m_Y;
        return xy_temp;
    };

    XY& operator/(double n) {
        static XY xy_temp;
        xy_temp.m_X = this->m_X / n;
        xy_temp.m_Y = this->m_Y / n;
        return xy_temp;
    };

    XY& operator+(XY& xy) {
        static XY xy_temp;
        xy_temp.m_X = this->m_X + xy.m_X;
        xy_temp.m_Y = this->m_Y + xy.m_Y;
        return xy_temp;
    }
};

// 车道线方程结构体  车道线方程在 车辆坐标系下  y=ax3+bx2+cx+d
struct LaneFunc
{
    float A_LaneFunc;
    float B_LaneFunc;  
    float C_LaneFunc;  
    float D_LaneFunc;  

    // LaneFunc();

    // LaneFunc(float a, float b, float c, float d):
    //         A_LaneFunc(a), B_LaneFunc(b), C_LaneFunc(c), D_LaneFunc(d) {};
};

// 车辆所在 车道 的左右车道线 编号
struct LaneNumber
{
    int LaneL;  // 左侧车道索引
    int LaneR;      // 右侧车道索引
};

// 枚举 车道类型 class  纯锥桶 纯车道线 混合车道
enum LaneType { PURECONE, MIXEDLANE, PURELANE };  // 0, 1, 2

// 锥桶在车道那一边
enum ConeSide {CONE_NULL, CONE_L, CONE_R};  // 0-不在边, 1-左边, 2-右边

enum ConeStatueByLane { INLANE, ONLANE, CONESTATUS_NULL};  // 0-车道内， 1-车道线上 2-无锥桶状态

// 场景类型
enum ScenarioType { 
    LANECHANGE, // 0-换道场景，
    PURELANEKEEP, // 1-纯车道 车道保持，
    CONELANEKEEP, // 2-锥桶车道与车道混合 车道保持，
    PURECONEKEEP, // 3-纯锥桶车道保持， 
    STOP }; //  4-停车场景

enum ScenarioStatus { 
    STATUS_UNKONW = 0,  //0-未知状态
    STATUS_PROCESSING =1,  // 1-正在处理，
    STATUS_DONE = 2 }; //  2-状态结束

struct Scene
{
    Eigen::VectorXd X_L_map;
    Eigen::VectorXd Y_L_map;

    Eigen::VectorXd X_M_map;
    Eigen::VectorXd Y_M_map;

    Eigen::VectorXd X_R_map;
    Eigen::VectorXd Y_R_map;

    Eigen::VectorXd X_C_map;
    Eigen::VectorXd Y_C_map;

};

// 德劳内三角剖分 后 边的中点结构体
struct DPoint
{
    double m_X;
    double m_Y;
    int C_Gap = 999;  // cost 点之间的间隔损失
    int C_Angle = 999;     // 夹角损失
    int C_Mid = 999;  // 靠近车辆坐标系x坐标轴 附近一定范围
    DPoint* m_parent; // 父节点的指针

    void setDefaultCost() {
        this->C_Angle = 999;
        this->C_Gap = 999;
        this->C_Mid = 999;
    };

    DPoint() = default;

    DPoint(double x, double y) {
        this->m_X = x;
        this->m_Y = y;
        this->m_parent = nullptr;
    }

    DPoint(double x, double y, DPoint* parent) {
        this->m_X = x;
        this->m_Y = y;
        this->m_parent = parent;
    }
};

// 德劳内三角剖分 边的定点索引结构体 一个边有 0， 1 点构成， 边就为（0，1）
struct Edge
{
    int first; // 第一个点的索引
    int last;  // 第二个点的索引
};

// 车辆状态
struct VehicleState
{
    double X;
    double Y;
    double yaw;
    double V;

    VehicleState() = default;

    VehicleState(double x, double y, double Yaw, double v):
                X(x), Y(y), yaw(Yaw), V(v) {};

    void updateState(double x, double y, double Yaw, double v) {
        this->X = x;
        this->Y = y;
        this->yaw = Yaw;
        this->V  = v;
    }
};

// 车辆动力学模型 状态矩阵
struct ModelMatrix
{
    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 1> B;
    Eigen::Matrix<double, 4, 1> C;
};

struct Path
{
    XY xy;
    double cur;
    double yaw_des;
};

enum MergeType { 
    FIVE_FIVE = 0,  //0-五阶贝塞尔拼五阶贝塞尔
    FIVE_THREE =1,  //  1-五阶贝塞尔拼三阶贝塞尔
    NO_MERGE = 2};  //  2-无需拼接

#endif /*_CONFIG_*/