#include "planning/lanekeep.h"
#include "eigen3/Eigen/Dense"
#include "External/Json/include/nlohmann/json.hpp"
#include "External/matplotlib/matplotlibcpp.h"
#include <fstream>

using json = nlohmann::json;

namespace plt = matplotlibcpp;

int main() {
    
    std::string conelanescene_file = "/home/syn/毕业设计/路径规划/Project/data/ConeLanekeepscene.json";


    std::ifstream ifs_conelane(conelanescene_file);


    json json_conelane;


    ifs_conelane >> json_conelane;

    std::vector<double> direct_map_L_X = json_conelane["LaneKeepScene"][0]["X_Direct_L"];
    std::vector<double> direct_map_L_Y = json_conelane["LaneKeepScene"][1]["Y_Direct_L"];
    std::vector<double> direct_map_cone_R_X = json_conelane["LaneKeepScene"][2]["X_Direct_cone_R"];
    std::vector<double> direct_map_cone_R_Y = json_conelane["LaneKeepScene"][3]["Y_Direct_cone_R"];

    std::vector<double> curve_map_L_X = json_conelane["LaneKeepScene"][4]["X_Curve_L"];
    std::vector<double> curve_map_L_Y = json_conelane["LaneKeepScene"][5]["Y_Curve_L"];
    std::vector<double> curve_map_cone_R_X = json_conelane["LaneKeepScene"][6]["X_Curve_cone_R"];
    std::vector<double> curve_map_cone_R_Y = json_conelane["LaneKeepScene"][7]["Y_Curve_cone_R"];


    Eigen::VectorXd direct_map_cone_r_x = Eigen::Map<Eigen::VectorXd>(direct_map_cone_R_X.data(), direct_map_cone_R_X.size());
    Eigen::VectorXd direct_map_cone_r_y = Eigen::Map<Eigen::VectorXd>(direct_map_cone_R_Y.data(), direct_map_cone_R_Y.size());

    Eigen::VectorXd curve_map_cone_r_x = Eigen::Map<Eigen::VectorXd>(curve_map_cone_R_X.data(), curve_map_cone_R_X.size());
    Eigen::VectorXd curve_map_cone_r_y = Eigen::Map<Eigen::VectorXd>(curve_map_cone_R_Y.data(), curve_map_cone_R_Y.size());

    Eigen::VectorXd curve_map_r_x = Eigen::Map<Eigen::VectorXd>(curve_map_L_X.data(), curve_map_L_X.size());
    Eigen::VectorXd curve_map_r_y = Eigen::Map<Eigen::VectorXd>(curve_map_L_Y.data(), curve_map_L_Y.size());


    std::vector<XY> direct_cone_XY;
    std::vector<XY> curve_cone_XY;

    for (int i = 0; i<direct_map_cone_r_x.size(); ++i) {
        XY conexy;
        conexy.m_X = direct_map_cone_r_y[i];
        conexy.m_Y = direct_map_cone_r_x[i];

        direct_cone_XY.push_back(conexy);
    }

    for (int i = 0; i<curve_map_cone_r_x.size(); ++i) {
        XY conexy;
        conexy.m_X = curve_map_cone_r_y[i];
        conexy.m_Y = curve_map_cone_r_x[i];

        curve_cone_XY.push_back(conexy);
    }

    ConeSide conside = CONE_L;
    std::vector<LaneFunc> nearlane_direct;
    LaneFunc leftlane;
    leftlane.A_LaneFunc = 0;
    leftlane.B_LaneFunc = 0;
    leftlane.C_LaneFunc = 0;
    leftlane.D_LaneFunc = 8.75;
    nearlane_direct.push_back(leftlane);

    LaneFunc rigehtfunc;
    rigehtfunc.A_LaneFunc = 0;
    rigehtfunc.B_LaneFunc = 0;
    rigehtfunc.C_LaneFunc = 0;
    rigehtfunc.D_LaneFunc = 5;
    nearlane_direct.push_back(rigehtfunc);

    ConeLaneKeep conelankeeper(nearlane_direct, direct_cone_XY, conside);
    std::vector<Path> direct_traj =  conelankeeper.plan();

    std::vector<double> conelankeep_direct_traj_x;
    std::vector<double> conelankeep_direct_traj_y;
    std::vector<double> conelane_direct_curvature;

    for (std::vector<Path>::iterator it = direct_traj.begin();
        it!= direct_traj.end();
        ++it) {
            conelankeep_direct_traj_x.push_back(it->xy.m_X);
            conelankeep_direct_traj_y.push_back(it->xy.m_Y);
            conelane_direct_curvature.push_back(it->cur);
        }

    // std::cout << conelankeep_direct_traj_y.size()<<'\n';

    auto coeff = polyfit(curve_map_r_y, curve_map_r_x, 3);
    
    // std::cout<< coeff<<'\n';

    LaneFunc lane;
    lane.A_LaneFunc = coeff[3];
    lane.B_LaneFunc = coeff[2];
    lane.C_LaneFunc = coeff[1];
    lane.D_LaneFunc = coeff[0];

    // LaneFunc lane1;
    // lane.A_LaneFunc = 0;
    // lane.B_LaneFunc = 0;
    // lane.C_LaneFunc =0;
    // lane.D_LaneFunc =2;

    std::vector<LaneFunc> curev_lane;
    curev_lane.push_back(lane);
    curev_lane.push_back(lane);

    ConeLaneKeep conelanekeeper_curve(curev_lane, curve_cone_XY, CONE_L);

    std::vector<Path> curve_traj =  conelanekeeper_curve.plan();

    std::vector<double> conelankeep_curve_traj_x;
    std::vector<double> conelankeep_curve_traj_y;
    std::vector<double> curve_curvature;

    for (std::vector<Path>::iterator it = curve_traj.begin();
        it!= curve_traj.end();
        ++it) {
            conelankeep_curve_traj_x.push_back(it->xy.m_X);
            conelankeep_curve_traj_y.push_back(it->xy.m_Y);
            curve_curvature.push_back(it->cur);
    }



    // zhixian
    plt::figure(1);
    plt::named_plot("cone",direct_map_cone_R_Y,direct_map_cone_R_X,"ro");
    plt::named_plot("lane",direct_map_L_Y, direct_map_L_X,"k-");
    plt::named_plot("planned path",conelankeep_direct_traj_x,conelankeep_direct_traj_y, "b-");

    // plt::scatter_colored(direct_map_cone_R_Y, direct_map_cone_R_X, "red",5);

    plt::xlabel("X(m)");
    plt::ylabel("Y(m)");
    plt::legend();
    plt::axis("equal");
    plt::show();

    // plt::figure(2);
    // plt::named_plot("curvature", conelankeep_direct_traj_x, conelane_direct_curvature,"r-");
    // plt::legend();
    // plt::xlabel("X(m)");
    // plt::ylabel("K(m-1)");
    // // plt::axis("equal");
    // plt::show();

    //wandao
    // plt::figure(1);
    // plt::named_plot("cone",curve_map_cone_R_Y,curve_map_cone_R_X,"ro");
    // plt::named_plot("lane",curve_map_L_Y, curve_map_L_X,"k-");
    // plt::named_plot("planned path",conelankeep_curve_traj_x,conelankeep_curve_traj_y, "b-");

    // // plt::scatter_colored(direct_map_cone_R_Y, direct_map_cone_R_X, "red",5);

    // plt::xlabel("X(m)");
    // plt::ylabel("Y(m)");
    // plt::legend();
    // plt::axis("equal");
    // plt::show();

    // plt::figure(2);
    // plt::named_plot("curvature", conelankeep_curve_traj_x, curve_curvature,"r-");
    // plt::legend();
    // plt::xlabel("X(m)");
    // plt::ylabel("K(m-1)");
    // // plt::axis("equal");
    // plt::show();
    return 0;
}