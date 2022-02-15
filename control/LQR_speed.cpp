#include "LQR_speed.h"

/**
 * 输入:
 * 输出：
 * 作用：  
**/
LQRSpeed::LQRSpeed(Eigen::Matrix<double, 5, 5> q,
            Eigen::Matrix<double, 2, 2> r,
            double v, double car_L):
            Q(q),
            R(r) {
    
    this->A.setZero();
    this->B.setZero();

    this->A(0,0) = 1.0;
    this->A(0,1) = ts;
    this->A(1,2) = v;
    this->A(2,2) = 1.0;
    this->A(2,3) = ts;
    this->A(4,4) = 1.0;

    this->B(3,0) = v / car_L;
    this->B(4,1) = ts;
}

/**
 * 输入:
 * 输出：
 * 作用：  
**/
Eigen::Matrix<double, 2, 5> LQRSpeed::compute_K() {
    int maxIter = 150;
    double eps = 0.001;

    Eigen::Matrix4d P_old = this->Q;

    for (int i = 0; i< maxIter; ++i) {
        Eigen::Matrix4d P_new = this->A.transpose() * P_old * this->A - 
                                this->A.transpose() * P_old * this->B *
                                (this->R + this->B.transpose() * P_old * this->B).inverse() *
                                this->B.transpose() * P_old * this->A + this->Q;
        if (fabs((P_new - P_old).maxCoeff()) < eps) {
            P_old = P_new;
            break;
        }
        P_old = P_new;
    }

    Eigen::Matrix<double, 2, 5> K = (this->B.transpose() *P_old * this->B + this->R).inverse() *
                                    (this->B.transpose() * P_old * this->A);

    return K;
};

/**
 * 输入:
 * 输出：
 * 作用：  
**/
size_t LQRSpeed::findNearsetPoint(VehicleState v_state) {
    size_t index = 0;
    double dis_min = std::numeric_limits<double>::max();

    for (std::vector<Path>::iterator it = this->ref_path.begin();
            it != this->ref_path.end();
                ++it) {
        double dx = v_state.X - it->xy.m_X;
        double dy = v_state.Y - it->xy.m_Y;

        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < dis_min) {
            dis_min = dist;
            index = std::distance(this->ref_path.begin(), it);
        }
    }
    return index;
}

/**
 * 输入:
 * 输出：
 * 作用：  
**/
void LQRSpeed::setPath(std::vector<Path> path) {
    this->ref_path = path;
}

/**
 * 输入:
 * 输出：
 * 作用：  
**/
std::array<double, 2> LQRSpeed::runLQRSpeed(VehicleState v_state, double target_speed) {
    size_t index = findNearsetPoint(v_state);

    double e1 = std::sqrt( std::pow((v_state.X - this->ref_path[index].xy.m_X), 2) + 
                            std::pow((v_state.Y - this->ref_path[index].xy.m_Y) ,2));

    double e2 = Pi2Pi(v_state.yaw - this->ref_path[index].yaw_des);

    double e1_d = (e1 - this->e1_old) / ts;
    double e2_d = (e2 - this->e2_old) / ts;
    double e3 = v_state.V - target_speed;

    Eigen::Matrix<double, 5, 1> state;
    state << e1,
                e1_d,
                e2,
                e2_d,
                e3;
    Eigen::Matrix<double, 2, 1> controlCommand = -compute_K() * state;

    double car_L = 2; // change!!!!!

    double feedforward = std::atan2(car_L * this->ref_path[index].cur, 1);
    double feedback = Pi2Pi(controlCommand(0,0));

    double steerangle = feedback+ feedforward;
    double accel = controlCommand(1,0);

    std::array<double, 2> command;
    command[0] = steerangle;
    command[1] = accel; 
    return command;
};