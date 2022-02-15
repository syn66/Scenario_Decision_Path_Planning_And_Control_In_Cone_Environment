#include "planning/lanekeep.h"
#include "External/Json/include/nlohmann/json.hpp"
#include "External/matplotlib/matplotlibcpp.h"
#include "eigen3/Eigen/Dense"
#include <fstream>

using json = nlohmann::json;
namespace plt = matplotlibcpp;

int main() {
    
    std::string purelanescene_file = "/home/syn/毕业设计/路径规划/Project/data/Lanekeepscene.json";
    std::string conelanescene_file = "/home/syn/毕业设计/路径规划/Project/data/ConeLanekeepscene.json";

    std::ifstream ifs_purelane(purelanescene_file);
    std::ifstream ifs_conelane(conelanescene_file);

    json json_purelane;
    json json_conelane;

    ifs_purelane >> json_purelane;
    ifs_conelane >> json_conelane;

    std::vector<double> direct_map_pure_L_X = json_purelane["LaneKeepScene"][0]["X_Direct_L"];
    std::vector<double> direct_map_pure_L_Y = json_purelane["LaneKeepScene"][1]["Y_Direct_L"];
    std::vector<double> direct_map_pure_R_X = json_purelane["LaneKeepScene"][2]["X_Direct_R"];
    std::vector<double> direct_map_pure_R_Y = json_purelane["LaneKeepScene"][3]["Y_Direct_R"];

    std::vector<double> curve_map_pure_L_X = json_purelane["LaneKeepScene"][4]["X_Curve_L"];
    std::vector<double> curve_map_pure_L_Y = json_purelane["LaneKeepScene"][5]["Y_Curve_L"];
    std::vector<double> curve_map_pure_R_X = json_purelane["LaneKeepScene"][6]["X_Curve_R"];
    std::vector<double> curve_map_pure_R_Y = json_purelane["LaneKeepScene"][7]["Y_Curve_R"];



    LaneFunc left_direct;
    LaneFunc rigth_direct;

    left_direct.A_LaneFunc = 0;
    left_direct.B_LaneFunc = 0;
    left_direct.C_LaneFunc = 0;
    left_direct.D_LaneFunc = 5;

    rigth_direct.A_LaneFunc = 0;
    rigth_direct.B_LaneFunc = 0;
    rigth_direct.C_LaneFunc = 0;
    rigth_direct.D_LaneFunc = 8.75;

    std::vector<LaneFunc> nearlane_direct;
    nearlane_direct.push_back(left_direct);
    nearlane_direct.push_back(rigth_direct);

    // direct//
    //
    //
    //
    //
    PureLaneKeep purelanekeeper(nearlane_direct);
    std::vector<XY> purelankeep_direct_traj = purelanekeeper.plan();

    std::vector<double> purelankeep_direct_traj_x;
    std::vector<double> purelankeep_direct_traj_y;
    std::vector<double> curevature_direct;  //qulv

    for (std::vector<XY>::iterator it = purelankeep_direct_traj.begin();
        it!= purelankeep_direct_traj.end();
        ++it) {
            purelankeep_direct_traj_x.push_back(it->m_X);
            purelankeep_direct_traj_y.push_back(it->m_Y);
            curevature_direct.push_back(0);
        }

    

    Eigen::VectorXd cure_map_L_X = Eigen::Map<Eigen::VectorXd>(curve_map_pure_L_Y.data(), curve_map_pure_L_Y.size());
    Eigen::VectorXd cure_map_L_y = Eigen::Map<Eigen::VectorXd>(curve_map_pure_L_X.data(), curve_map_pure_L_X.size());
    Eigen::VectorXd cure_map_R_X = Eigen::Map<Eigen::VectorXd>(curve_map_pure_R_Y.data(), curve_map_pure_R_Y.size());
    Eigen::VectorXd cure_map_R_Y = Eigen::Map<Eigen::VectorXd>(curve_map_pure_R_X.data(), curve_map_pure_R_X.size());


    auto coeff_L =  polyfit(cure_map_L_X, cure_map_L_y,3);
    auto coeff_R =  polyfit(cure_map_R_X, cure_map_R_Y,3);

    LaneFunc cure_L;
    LaneFunc cure_R;
    cure_L.A_LaneFunc = coeff_L[3];
    cure_L.B_LaneFunc = coeff_L[2];
    cure_L.C_LaneFunc = coeff_L[1];
    cure_L.D_LaneFunc = coeff_L[0];

    cure_R.A_LaneFunc = coeff_R[3];
    cure_R.B_LaneFunc = coeff_R[2];
    cure_R.C_LaneFunc = coeff_R[1];
    cure_R.D_LaneFunc = coeff_R[0];

    std::vector<LaneFunc> near_curev_lane;
    near_curev_lane.push_back(cure_L);
    near_curev_lane.push_back(cure_R);

    // curve
    //
    //
    //

    PureLaneKeep purelanekeeper_curve( near_curev_lane);

    std::vector<XY> cure_traj =  purelanekeeper_curve.plan();


    std::vector<double> purelankeep_curve_traj_x;
    std::vector<double> purelankeep_curve_traj_y;
    std::vector<double> curvature_curve;
    std::vector<double> curvature_curve_x;
    std::vector<double> curve_traj_x;
    std::vector<double> curve_traj_y;


    for (std::vector<XY>::iterator it = cure_traj.begin();
        it!= cure_traj.end();
        ++it) {
            purelankeep_curve_traj_x.push_back(it->m_X);
            purelankeep_curve_traj_y.push_back(it->m_Y);
        }

    Eigen::VectorXd cure_mid_X = Eigen::Map<Eigen::VectorXd>(purelankeep_curve_traj_x.data(), purelankeep_curve_traj_x.size());
    Eigen::VectorXd cure_mid_Y = Eigen::Map<Eigen::VectorXd>(purelankeep_curve_traj_y.data(), purelankeep_curve_traj_y.size());

    auto coeff_midlane = polyfit(cure_mid_X,cure_mid_Y,3);  // y = d + cx + bx2 + ax3

    for(double i=0; i<=21.5; i+=0.1) {
        double k = fabs(6 * coeff_midlane[3] * i + 2 * coeff_midlane[2]) / 
                    std::pow( (1 + std::pow( (3* coeff_midlane[3] * i*i +2 *coeff_midlane[2] * i + coeff_midlane[1]), 2)) ,1.5);
        curvature_curve.push_back(k);
        curvature_curve_x.push_back(i);

        curve_traj_x.push_back(i);
        curve_traj_y.push_back(coeff_midlane[3] *i*i*i + coeff_midlane[2] *i*i + coeff_midlane[1]*i + coeff_midlane[0]);


    };


////plot
///
    plt::figure(1);

    // plt::plot(direct_map_pure_L_Y, direct_map_pure_L_X, "k-");
    plt::plot(direct_map_pure_R_Y, direct_map_pure_R_X, "k-");
    // plt::plot(purelankeep_direct_traj_x, purelankeep_direct_traj_y,"r-");
    plt::xlabel("X(m)");
    plt::ylabel("Y(m)");
    plt::named_plot("lane",direct_map_pure_L_Y,direct_map_pure_L_X, "k-");
    plt::named_plot("planned path",purelankeep_direct_traj_x,purelankeep_direct_traj_y,"r-");
    plt::legend();
    // plt::ylim(0,12);
    // plt::xlim(0,30);
    plt::axis("equal");
    plt::show();


    plt::figure(2);
    // plt::plot(curve_map_pure_L_Y, curve_map_pure_L_X, "b-");
    plt::plot(curve_map_pure_R_Y, curve_map_pure_R_X, "k-");
    // plt::plot(curve_traj_x, curve_traj_y, "r-");
    plt::xlabel("X(m)");
    plt::ylabel("Y(m)");
    plt::named_plot("lane",curve_map_pure_L_Y,curve_map_pure_L_X,"k-");
    plt::named_plot("planned path", curve_traj_x,curve_traj_y,"r-");
    plt::legend();
    plt::ylim(0,30);
    plt::xlim(0,30);
    plt::axis("equal");
    plt::show();

    // plt::rcparams({"font.SimHei","axes.unicode_minus"});

    plt::figure(3);
    // plt::plot(purelankeep_direct_traj_x, curevature_direct, "k-");
    plt::named_plot("curvature", purelankeep_direct_traj_x,curevature_direct,"r-");

    plt::ylabel("k(m-1)");
    plt::xlabel("X(m)");
    plt::legend();
    plt::show();

    plt::figure(4);
    plt::ylabel("k(m-1)");
    plt::xlabel("X(m)");
    plt::named_plot("curvature",curvature_curve_x,curvature_curve, "r-");
    plt::legend();
    plt::show();

    return 0;
}