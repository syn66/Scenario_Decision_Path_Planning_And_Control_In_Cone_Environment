#include "planning/delaunay_planner.h"
#include "Plotting/plotting.h"

#include "External/Json/single_include/nlohmann/json.hpp"

// #include "SturmSequence.hpp"
// #include "Bezier.hpp"

#include <fstream>

using json = nlohmann::json;

int main() {

    std::fstream ifs_cone("/home/syn/毕业设计/路径规划/Project/data/cone.json");
    json json_cone;
    ifs_cone >> json_cone;

    std::vector<double> direct_X_L = json_cone["cone"][0]["direct_x_L"];
    std::vector<double> direct_Y_L = json_cone["cone"][1]["direct_y_L"];
    std::vector<double> direct_X_R = json_cone["cone"][2]["direct_x_R"];
    std::vector<double> direct_Y_R = json_cone["cone"][3]["direct_y_R"];
    
    std::vector<double> curve_X_L = json_cone["cone"][4]["curve_x_L"];
    std::vector<double> curve_Y_L = json_cone["cone"][5]["curve_y_L"];
    std::vector<double> curve_X_R = json_cone["cone"][6]["curve_x_R"];
    std::vector<double> curve_Y_R = json_cone["cone"][7]["curve_y_R"];

    Eigen::VectorXd direct_x_l = Eigen::Map<Eigen::VectorXd>(direct_X_L.data(), direct_X_L.size());
    Eigen::VectorXd direct_y_l = Eigen::Map<Eigen::VectorXd>(direct_Y_L.data(), direct_Y_L.size());
    Eigen::VectorXd direct_x_r = Eigen::Map<Eigen::VectorXd>(direct_X_R.data(), direct_X_R.size());
    Eigen::VectorXd direct_y_r = Eigen::Map<Eigen::VectorXd>(direct_Y_R.data(), direct_Y_R.size());

    Eigen::VectorXd curve_x_l = Eigen::Map<Eigen::VectorXd>(curve_X_L.data(), curve_X_L.size());
    Eigen::VectorXd curve_y_l = Eigen::Map<Eigen::VectorXd>(curve_Y_L.data(), curve_Y_L.size());
    Eigen::VectorXd curve_x_r = Eigen::Map<Eigen::VectorXd>(curve_X_R.data(), curve_X_R.size());
    Eigen::VectorXd curve_y_r = Eigen::Map<Eigen::VectorXd>(curve_Y_R.data(), curve_Y_R.size());

    std::vector<XY> direct_cone;
    std::vector<XY> curve_cone;

    for (int i = 0; i<direct_x_l.size(); ++i) {
        XY xy;
        xy.m_X = direct_x_l[i];
        xy.m_Y = direct_y_l[i];
        direct_cone.push_back(xy);
    }

    for (int i = 0; i<direct_x_r.size(); ++i) {
        XY xy;
        xy.m_X = direct_x_r[i];
        xy.m_Y = direct_y_r[i];
        direct_cone.push_back(xy);
    }

    for (int i = 0; i<curve_x_l.size(); ++i) {
        XY xy;
        xy.m_X = curve_x_l[i];
        xy.m_Y = curve_y_l[i];
        curve_cone.push_back(xy);
    }   

    for (int i = 0; i<curve_x_r.size(); ++i) {
        XY xy;
        xy.m_X = curve_x_r[i];
        xy.m_Y = curve_y_r[i];
        curve_cone.push_back(xy);
    }


    // 直线工况测试
    Delaunay_Planner delaunay_plan_direct(direct_cone);

    std::vector<Path> direct_path = delaunay_plan_direct.Plan();

    std::vector< std::pair<Point, unsigned> > direct_point = delaunay_plan_direct.getPoints();

    std::vector<Edge> direct_edge = delaunay_plan_direct.getEdgevec();

    std::vector<DPoint> direct_midpoint = delaunay_plan_direct.getMidPoints();

    Plotting plotter;
    plotter.plotDelaunay_planning( direct_point, direct_edge, direct_midpoint, direct_path);

    std::vector<double> curvature;
    std::vector<double> x;
    for (std::vector<Path>::iterator it = direct_path.begin(); it!=direct_path.end(); ++it) {
        double cur = it->cur;
        x.push_back(it->xy.m_X);
        curvature.push_back(cur);
    }

    plt::figure(2);
    plt::named_plot("curvature", x,curvature,"r-");
    plt::legend();
    plt::xlabel("X(m)");
    plt::ylabel("K(m-1)");
    // plt::axis("equal");
    plt::show();



    // // 弯道工况测试
    // Delaunay_Planner delaunay_plan_curve(curve_cone);

    // std::vector<Path> curve_path = delaunay_plan_curve.Plan();

    // std::vector< std::pair<Point, unsigned> > curve_point = delaunay_plan_curve.getPoints();
    
    // std::vector<Edge> curve_edge = delaunay_plan_curve.getEdgevec();

    // std::vector<DPoint> curve_midpoint = delaunay_plan_curve.getMidPoints();

    // Plotting plotter1;
    // plotter1.plotDelaunay_planning( curve_point, curve_edge, curve_midpoint, curve_path);

    // std::vector<double> curvature;
    // std::vector<double> x;
    // for (std::vector<Path>::iterator it = curve_path.begin(); it!=curve_path.end(); ++it) {
    //     double cur = it->cur;
    //     x.push_back(it->xy.m_X);
    //     curvature.push_back(cur);
    // }

    // // plt::figure(1);
    // // plt::plot(x,curvature);
    // // plt::show();

    // plt::figure(2);
    // plt::named_plot("curvature", x,curvature,"r-");
    // plt::legend();
    // plt::xlabel("X(m)");
    // plt::ylabel("K(m-1)");
    // // plt::axis("equal");
    // plt::show();

    // std::vector<double> sss;
    // for (int i =0; i<10;++i) {
    //     sss.push_back(i);
    // }

    // plt::figure(3);
    // plt::named_plot("cone",sss,sss,"gD");
    // plt::named_plot("Delaunay triangle",sss,sss,"g--");
    // plt::named_plot("planned path",sss,sss,"b-");
    // plt::named_plot("midpoint",sss,sss,"ro");

    // plt::legend();
    // plt::show();

    return 0;
}