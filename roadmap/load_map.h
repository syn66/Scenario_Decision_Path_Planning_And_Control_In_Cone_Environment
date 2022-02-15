#ifndef _LOADMAP_
#define _LOADMAP_

#include <iostream>
#include <fstream>
#include <vector>
#include <assert.h>

#include "External/Json/include/nlohmann/json.hpp"

#include "../config.h"
#include "../utils.h"

using json = nlohmann::json;



class LoadMap
{
private:

    static const int PointHorizon;  //暂定 后续调参 取点范围
    // int index_L = 0;  // 左侧车道线 地图点索引
    int index = 0; //  中间车道线 地图点索引
    // int index_R = 0; //  右侧车道线 地图点索引

    XY car_location;  // 车辆当前位置 通过外部传入参数 更新
    double car_head_angle;  // 车辆当前横摆角 通过外部传入参数更新

    // 全局坐标系下
    Eigen::VectorXd X_L_vec;
    Eigen::VectorXd Y_L_vec;

    Eigen::VectorXd X_M_vec;
    Eigen::VectorXd Y_M_vec;

    Eigen::VectorXd X_R_vec;
    Eigen::VectorXd Y_R_vec;

    Eigen::VectorXd X_C_vec;
    Eigen::VectorXd Y_C_vec;

    // 车辆坐标系下
    std::vector<XY> Coor_C_Horizon;  //zhuitong zuobiao
    std::vector<LaneFunc> PolyFitLaneFunc; // chedaoxianfangcheng vector

    Eigen::VectorXd X_L_horzion;
    Eigen::VectorXd Y_L_horzion;

    Eigen::VectorXd X_M_horzion;
    Eigen::VectorXd Y_M_horzion;

    Eigen::VectorXd X_R_horzion;
    Eigen::VectorXd Y_R_horzion;

    void init( std::string road_map_file);

    void assignthorizon();

    void polyfitLanefuc();

public:

    LoadMap( std::string road_map_file);

    std::vector<LaneFunc>& getCurrentlaneFunc();

    std::vector<XY>& getCurrentConeCoor();

    const XY& getcurrentCarlocation();

    void updateCurrentMap( XY& car_location_return, double &phi );

    const Scene& getMap();

};




#endif /*_LOADMAP_*/