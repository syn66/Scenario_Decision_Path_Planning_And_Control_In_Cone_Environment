#pragma once

#include <vector>

#include "../config.h"
#include "../utils.h"
#include "bezier.h"

// 纯车道线车道保持 路径规划
class PureLaneKeep 
{
private:

    static int Sample_num_X; // 车道线采样点的个数 沿车辆坐标系X轴
    static int Sample_step_X; //车道线采样间隔

    std::vector<LaneFunc> near_Lane;  // 车辆两侧车道线方程 0-左，1-右

public:
    PureLaneKeep( std::vector<LaneFunc> nearLane );

    std::vector<XY> plan(); 
};


// 锥桶车道线混合车道保持 路径规划   锥桶间距为5-6m
class ConeLaneKeep
{
private:
    std::vector<LaneFunc> near_Lane;  // 车辆两侧车道线方程

    std::vector<XY> cone_point; //锥桶在车辆坐标系下的坐标

    ConeSide current_cone_side = CONE_NULL; // 锥桶在哪一侧车道

public:
    ConeLaneKeep( std::vector<LaneFunc> nearlane, std::vector<XY> cone_XY, ConeSide conesdie);

    std::vector<Path> plan();
};