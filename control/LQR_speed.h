#pragma once

#include "../config.h"
#include "../utils.h"

class LQRSpeed
{
private:

    Eigen::Matrix<double, 5, 5> A;
    Eigen::Matrix<double, 5, 2> B;

    Eigen::Matrix<double, 5, 5> Q;
    Eigen::Matrix<double, 2, 2> R;
    
    double e1_old = 0;
    double e2_old = 0;

    std::vector<Path> ref_path;

    Eigen::Matrix<double, 2, 5> compute_K();

    size_t findNearsetPoint(VehicleState v_state);

public:
    LQRSpeed(Eigen::Matrix<double, 5, 5> q,
            Eigen::Matrix<double, 2, 2> r,
            double v, double car_L);

    void setPath(std::vector<Path> path);

    std::array<double, 2> runLQRSpeed(VehicleState v_state, double target_speed);
};