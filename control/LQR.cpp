#include "LQR.h"

/**
 * 输入： 无
 * 输出： 无
 * 作用： 构造函数
**/
LQRcontrol::LQRcontrol( VehicleDynamicsModel &vehicle_Model, 
                        Eigen::Matrix<double, 4, 4> &q,
                        Eigen::Matrix<double, 1, 1> &r):
                        Model(vehicle_Model),
                        Q(q),
                        R(r) {};

/**
 * 输入： 无
 * 输出： 无
 * 作用： 求解黎卡提方程组求解反馈矩阵 K
**/
Eigen::Matrix<double, 1, 4> LQRcontrol::ComputeK() {
    int maxloop = 200;
    double minestVal = 10e-10;

    Eigen::Matrix4d P_old;
    P_old = this->Q;

    for (int i = 0; i<maxloop; ++i) {
        // 黎卡提方程组 求新的P
        Eigen::Matrix4d P_new = this->Q +
                                this->Model.getdisModeltrix()->A.transpose() * P_old * this->Model.getdisModeltrix()->A -
                                this->Model.getdisModeltrix()->A.transpose() * P_old * this->Model.getdisModeltrix()->B * 
                                (this->R + this->Model.getdisModeltrix()->B.transpose() * P_old *this->Model.getdisModeltrix()->B).inverse() * 
                                this->Model.getdisModeltrix()->B.transpose() * P_old * this->Model.getdisModeltrix()->A;
        // 满足最小 便跳出循环 maxcoeff 求最大系数
        if (fabs((P_new - P_old).maxCoeff()) < minestVal) {
            P_old = P_new;
            break;
        }
        P_old = P_new;
    }
    // 反馈矩阵 K
    Eigen::Matrix<double, 1, 4> K;

    K = -(this->R + this->Model.getdisModeltrix()->B.transpose() * 
            P_old * this->Model.getdisModeltrix()->B).inverse() * 
            this->Model.getdisModeltrix()->B.transpose() * P_old * this->Model.getdisModeltrix()->A; 
    return K;
};

/**
 * 输入： 无
 * 输出： 无
 * 作用： 计算车辆当前与轨迹的 误差  状态矩阵
**/
Eigen::Matrix<double, 4, 1> LQRcontrol::ComputeState(VehicleState v_state, size_t index) {
    Eigen::Matrix<double, 4, 1> state;
    
    double e1 = 0;
    double e2 = 0;

    double dx = v_state.X - this->path[index].xy.m_X;
    double dy = v_state.Y - this->path[index].xy.m_Y;

    double dist = std::sqrt((dx*dx + dy*dy));

    Eigen::Matrix<double, 2, 1> rotation_matrix;
    rotation_matrix << std::cos(v_state.yaw + M_PI / 2.0), 
                        std::sin(v_state.yaw + M_PI / 2.0);
    Eigen::Matrix<double, 1, 2> path_cg_matrix;
    path_cg_matrix << dx, dy;

    if (path_cg_matrix * rotation_matrix > 0.0) {
        e1 = 1.0 * dist;
    }
    else {
        e1 = -1.0 * dist;
    }

    double current_yaw = 0;
    if (v_state.yaw < 0) {
        current_yaw += 2*M_PI;
    }
    
    e2 = current_yaw - this->path[index].yaw_des;
    if (e2 > M_PI) {
        e2 -= 2.0 * M_PI;
    }
    else if (e2 < -M_PI) {
        e2 += 2.0 * M_PI;
    }

    state << e1, 
            (e1 - this->old_e1) / ts,
            e2,
            (e2 - this->old_e2) / ts;
    
    this->old_e1 = e1;
    this->old_e2 = e2;

    return state;
};

/**
 * 输入： 当前路径点 曲率 ，纵向速度
 * 输出： 前馈角
 * 作用： 计算前馈角  
**/
double LQRcontrol::ComputeFeedForward(double curvature, double V) {
    // 不足转向系数
    double Kv = this->Model.getKv();
    double forward_angle = (this->Model.getVehicleParam()(0,7) * curvature) + 
                            Kv * V * V * curvature - 
                            ComputeK()(0,2) * (this->Model.getVehicleParam()(0,4) * curvature - 
                            this->Model.getVehicleParam()(0,3) * this->Model.getVehicleParam()(0,1) * V * V *curvature / 2.0 /
                            this->Model.getVehicleParam()(0,6) / this->Model.getVehicleParam()(0,7));
    return forward_angle;
};  

/**
 * 输入： 无
 * 输出： 无
 * 作用： 找到当前匹配点最短距离的点
**/
size_t LQRcontrol::findPathref(double current_X, double current_Y) {

    size_t index = 0;
    double dis_min = std::numeric_limits<double>::max();

    for (std::vector<Path>::iterator it = this->path.begin();
        it != this->path.end();
        ++it) {
        
        double dis = std::pow(it->xy.m_X - current_X, 2) +
                    std::pow(it->xy.m_Y - current_Y, 2);
        if (dis < dis_min) {
            dis_min = dis;
            index = std::distance(this->path.begin(), it);
        }
    }
    return index;
};

/**
 * 输入： 无
 * 输出： 无
 * 作用： 找到当前匹配点
**/
void LQRcontrol::setPath(std::vector<Path> traj) {
    this->path = traj;
}

/**
 * 输入： 无
 * 输出： 无
 * 作用： 限制最大转向角
**/
double LQRcontrol::limitSteerangle(double angle, double min_angle, double max_angle) {
    double real_angle = 0;
    if (angle < min_angle) {
        real_angle = min_angle;
    }
    else if (angle > max_angle) {
        real_angle = max_angle;
    }
    else {
        real_angle = angle;
    }
    return real_angle;
}


/**
 * 输入： 无
 * 输出： 无
 * 作用： 找到当前匹配点
**/
std::array<double, 3> LQRcontrol::runLQR(VehicleState& v_state) {
    std::array<double, 3> command_current;
    double steer_angle;

    size_t index = findPathref(v_state.X, v_state.Y);

    Eigen::Matrix<double ,1 ,4> k = ComputeK();

    Eigen::Matrix<double, 4, 1> state = ComputeState(v_state, index);

    double feedforward_angle = ComputeFeedForward(this->path[index].cur, v_state.V);

    double feedback_angle = -k * state;

    std::cout<< "feedback  "<< feedback_angle << '\n';
    std::cout<< "feedward  "<< feedforward_angle << '\n';

    steer_angle = feedback_angle + feedforward_angle;
    steer_angle = limitSteerangle(steer_angle, -this->Model.getVehicleParam()(0,0), this->Model.getVehicleParam()(0,0));

    // double vx = v_state.V * std::cos(this->path[index].yaw_des);
    // this->Model.updateModel(vx);

    command_current[0] = steer_angle;
    command_current[1] = state(0,0);
    command_current[2] = state(2,0);
    
    return command_current;
};