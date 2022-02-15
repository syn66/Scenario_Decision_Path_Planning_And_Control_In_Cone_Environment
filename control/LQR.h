#pragma once

#include "eigen3/Eigen/Dense"

#include "../config.h"
#include "../model/dynamic_model.h"

class LQRcontrol
{
private:
    // 车辆动力学模型
    VehicleDynamicsModel Model;

    Eigen::Matrix<double, 4, 4> Q;
    Eigen::Matrix<double, 1, 1> R;

    double old_e1 = 0;
    double old_e2 = 0;
    // 全局坐标系下
    std::vector<Path> path;

    //计算 反馈矩阵 K
    Eigen::Matrix<double, 1, 4> ComputeK();
    //计算前馈
    double ComputeFeedForward(double curvature, double V);

    // 计算状态变量
    Eigen::Matrix<double, 4, 1> ComputeState(VehicleState v_state, size_t index);

    size_t findPathref(double current_X, double current_Y);

    double limitSteerangle(double angle, double min_angle, double max_angle);
    
public:
    LQRcontrol( VehicleDynamicsModel &vehicle_Model, 
                Eigen::Matrix<double, 4, 4> &q,
                Eigen::Matrix<double, 1, 1> &r);
    // 需要将车辆坐标系下的 轨迹 转换到 全局坐标系下
    void setPath(std::vector<Path> traj);

    std::array<double, 3> runLQR(VehicleState& v_state);
};