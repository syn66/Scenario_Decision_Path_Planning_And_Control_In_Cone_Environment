#ifndef _PLOTTING_
#define _PLOTTING_

#include <vector>
#include <fstream>
#include "../config.h"
#include "../utils.h"

#include "External/matplotlib/matplotlibcpp.h"
#include "External/Json/include/nlohmann/json.hpp"


namespace plt = matplotlibcpp;

using json = nlohmann::json;


class Plotting
{
private:
    const static double car_L;
    const static double car_W;
    const static double rear_axle;  // 后悬
    // XY last_car_Location; 

    void plotCar(const XY &carlocation ,const double &phi) const;
    void plotScenario(ScenarioType& st, XY& car_Location);

public:
    Plotting();

    void plotRun();
    // TODO 传入参数 添加LQR控制的实际轨迹log
    void plotSim(std::vector<XY> &trajectory, const Scene &scene,  XY &carlocation ,const double &phi);
    void plotDelaunay_planning( std::vector< std::pair<Point, unsigned> > points, 
                                std::vector<Edge>& edge_vec, 
                                std::vector<DPoint>& mid_point,
                                std::vector<Path> path);

};

#endif /*_PLOTTING_*/