#pragma once

#include "../config.h"

class PurePursuit
{
private:
    double car_L;

    double lookahead_dis;

    size_t target_point_index;

    bool isReachTargetPoint();

    void compute_new_lookahead_dis();

    void compute_targetPOint();

    double compute_delta();
    
public:
    PurePursuit(std::vector<Path>& ref_path, double L);

    double runPP(XY& car_loaction);
};