#include "dynamic_model.h"

/**
 * 输入： 无
 * 输出： 无
 * 作用： 构造函数
**/
VehicleDynamicsModel::VehicleDynamicsModel(double ts, double vx, std::string param_Path):
                                            Ts(ts) {
    std::ifstream ifs_param(param_Path); // 车辆参数文件路径
    json json_param;
    ifs_param >> json_param;

    this->maxSteer = json_param["maxSteer"];
    this->m = json_param["m"];
    this->Iz = json_param["Iz"];
    this->lf = json_param["lf"];
    this->lr = json_param["lr"];
    this->cf = json_param["cf"];
    this->cr = json_param["cr"];
    this->maxAccl = json_param["maxAccl"];
    this->l = this->lf + this->lr;

    InitModelmatrix(vx);
}

/**
 * 输入： 无
 * 输出： 无
 * 作用： 根据车辆参数 初始化车辆动力学模型 状态矩阵
**/
void VehicleDynamicsModel::InitModelmatrix(double Vx) {
    // 状态矩阵  a b c
    this->modelmatrix.A = Eigen::Matrix<double, 4, 4>::Zero();
    this->modelmatrix.B = Eigen::Matrix<double, 4, 1>::Zero();
    this->modelmatrix.C = Eigen::Matrix<double, 4, 1>::Zero();
    // 赋值
    this->modelmatrix.A(0,1) = 1;
    this->modelmatrix.A(1,1) = (cf+cr)/(m*Vx);
    this->modelmatrix.A(1,2) = -(cf+cr)/m;
    this->modelmatrix.A(1,3) = (lf*cf - lr*cr)/(m*Vx);
    this->modelmatrix.A(2,3) = 1;
    this->modelmatrix.A(3,1) = (lf*cf - lr*cr)/(Iz*Vx);
    this->modelmatrix.A(3,2) = -(lf*cf - lr*cr)/Iz;
    this->modelmatrix.A(3,3) = (lf*lf*cf+lr*lr*cr)/(Iz*Vx);

    this->modelmatrix.B(1,0) = -(cf)/m;
    this->modelmatrix.B(3,0) = -(lf*cf)/Iz;

    this->modelmatrix.C(1,0) = (lr*cr - lf*cf)/(m*Vx) - Vx;
    this->modelmatrix.C(3,0) = -(lf*lf*cf + lr*lr*cr)/(Iz*Vx);
};


/**
 * 输入： 无
 * 输出： 离散化后的modelmatrix 动力学状态矩阵
 * 作用： 使用零阶保持器 ZOH进行离散化
**/
ModelMatrix VehicleDynamicsModel::discretizeModel() {
    // 单位矩阵4x4
    Eigen::Matrix<double, 4, 4> eye;
    eye.setIdentity(4,4);

    ModelMatrix discretize_model;
    // ad = e(AT) = e(AT/2) / e(-AT/2) 分子分母泰勒展开 取前两项 为下式
    discretize_model.A = (eye - this->modelmatrix.A*this->Ts*0.5).inverse() * 
                            (eye + this->modelmatrix.A*this->Ts*0.5);
    discretize_model.B = this->modelmatrix.B * this->Ts;
    discretize_model.C = this->modelmatrix.C * this->Ts;
    return discretize_model;
};  

/**
 * 输入： 无
 * 输出： 无
 * 作用： 更新动力学模型
**/
void VehicleDynamicsModel::updateModel(double vx) {
    InitModelmatrix(vx);
};

/**
 * 输入： 无
 * 输出： modelmatrix 动力学状态矩阵
 * 作用： 接口 - 返回离散化后的车辆动力学模型 状态矩阵
**/
ModelMatrix* VehicleDynamicsModel::getdisModeltrix() {
    static ModelMatrix model = discretizeModel();
    ModelMatrix* bicyle_model_dis = &model;
    return bicyle_model_dis;
};

/**
 * 输入： 无
 * 输出： modelmatrix 动力学状态矩阵
 * 作用： 接口 - 返回离散化后的车辆动力学模型 状态矩阵
**/
double VehicleDynamicsModel::getKv() {
    double kv = this->lr * this->m / 2.0 / this->cf / this->l - 
                this->lf * this->m / 2.0 / this->cr / this->l;
    return kv;
}

/**
 * 输入： 无
 * 输出： modelmatrix 动力学状态矩阵
 * 作用： 接口 - 返回离散化后的车辆动力学模型 状态矩阵
**/
Eigen::Matrix<double, 1, 9> VehicleDynamicsModel::getVehicleParam() {
    Eigen::Matrix<double, 1, 9> vehicleparam;
    vehicleparam << this->maxSteer, this->m, this->Iz, this->lf, this->lr, this->cf, this->cr, this->l, this->maxAccl;
    return vehicleparam;
}