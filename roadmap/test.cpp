
#include <fstream>
#include <iostream>
#include "json.hpp"

#include "External/eigen-3.4.0/Eigen/Dense"

using json = nlohmann::json;

int main(){
    std::ifstream ifs("/home/syn/毕业设计/路径规划/project/roadmap/road_map.json");
    json jsontest;
    ifs >> jsontest;

    // std::cout<< jsontest["LaneAndConeCoordinate"][0]["X_l"] <<std::endl;

    Eigen::VectorXd xx;
    xx<< 1, 2,3,4,5,6;

    std::cout<< xx<<std::endl;
    Eigen::VectorXd( Eigen::seqN(3,2));

    Eigen::seqN(3,2);
};