#pragma once

#include "../config.h"
#include "../External/Json/single_include/nlohmann/json.hpp"
#include <fstream>

using json = nlohmann::json;

class VehicleDynamicsModel
{
private:
    // main.cpp 通过json配置文件读入  
    double maxSteer; // 前轮最大转角
    double maxAccl; //最大加速度

    double m;    // 质量
    double Iz;  // 转动惯量
    double lf;  //  前轴到重心距离
    double lr;   //后轴到重心距离
    double cf;  // 前轮侧偏刚度  
    double cr;  //   后轮侧偏刚度
    double l;

    ModelMatrix modelmatrix; //状态矩阵

    double Ts; // 离散化时间间隔

    void InitModelmatrix(double Vx);

    ModelMatrix discretizeModel();

public:
    VehicleDynamicsModel(double ts, double vx, std::string param_path);
    
    void updateModel(double vx);

    double getKv();

    ModelMatrix* getdisModeltrix();

    Eigen::Matrix<double, 1, 9> getVehicleParam();
};

