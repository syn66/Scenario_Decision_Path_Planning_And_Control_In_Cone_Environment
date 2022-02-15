#include "planning/quintic_polynomial.h"
#include "control/LQR.h"
#include "model/dynamic_model.h"
#include "Plotting/plotting.h"

#include "External/Json/single_include/nlohmann/json.hpp"
#include "External/matplotlib/matplotlibcpp.h"
#include "config.h"
#include "utils.h"
#include "control/pid.h"
#include "control/LQR_speed.h"


#include <fstream>

using json = nlohmann::json;
namespace plt = matplotlibcpp;

int main() {

    std::fstream ifs_lanechange("/home/syn/毕业设计/路径规划/Project/data/lanechange.json");
    json json_lc;
    ifs_lanechange >> json_lc;

    std::vector<double> X_L = json_lc["Lanechange"][0]["laneleft_X"];
    std::vector<double> Y_L = json_lc["Lanechange"][1]["laneleft_Y"];
    std::vector<double> X_R = json_lc["Lanechange"][2]["lanemiddle_X"];
    std::vector<double> Y_R = json_lc["Lanechange"][3]["lanemiddle_Y"];
    std::vector<double> X_M = json_lc["Lanechange"][4]["laneright_X"];
    std::vector<double> Y_M = json_lc["Lanechange"][5]["laneright_Y"];

    std::vector<double> C_X = json_lc["Lanechange"][6]["cone_X"];
    std::vector<double> C_Y = json_lc["Lanechange"][7]["cone_Y"];

    Eigen::VectorXd direct_x_l = Eigen::Map<Eigen::VectorXd>(X_L.data(), X_L.size());
    Eigen::VectorXd direct_y_l = Eigen::Map<Eigen::VectorXd>(Y_L.data(), Y_L.size());
    Eigen::VectorXd direct_x_m = Eigen::Map<Eigen::VectorXd>(X_M.data(), X_M.size());
    Eigen::VectorXd direct_y_m = Eigen::Map<Eigen::VectorXd>(Y_M.data(), Y_M.size());

    Eigen::VectorXd curve_x_r = Eigen::Map<Eigen::VectorXd>(X_R.data(), X_R.size());
    Eigen::VectorXd curve_y_r = Eigen::Map<Eigen::VectorXd>(Y_R.data(), Y_R.size());
    Eigen::VectorXd curve_c_x = Eigen::Map<Eigen::VectorXd>(C_X.data(), C_X.size());
    Eigen::VectorXd curve_c_y = Eigen::Map<Eigen::VectorXd>(C_Y.data(), C_Y.size());

    std::vector<XY> laneleft;
    std::vector<XY> laneright;
    std::vector<XY> lanemiddle;

    std::vector<XY> cone;

    for (int i = 0; i<direct_x_l.size(); ++i) {
        XY xy;
        xy.m_X = direct_x_l[i];
        xy.m_Y = direct_y_l[i];
        laneleft.push_back(xy);
    }

    for (int i = 0; i<direct_x_m.size(); ++i) {
        XY xy;
        xy.m_X = direct_x_m[i];
        xy.m_Y = direct_y_m[i];
        lanemiddle.push_back(xy);
    }
    
    for (int i = 0; i<curve_x_r.size(); ++i) {
        XY xy;
        xy.m_X = curve_x_r[i];
        xy.m_Y = curve_y_r[i];
        laneright.push_back(xy);
    }

    for (int i = 0; i<curve_c_x.size(); ++i) {
        XY xy;
        xy.m_X = curve_c_x[i];
        xy.m_Y = curve_c_y[i];
        cone.push_back(xy);
    }

    QuinticPolynomial  quinticer(0, 5, 0.1, 
                                    20, 5, 0.1,
                                    6.875, 0, 0,
                                    10.625,0,0,
                                    4);
    
    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> yaw;

    std::vector<Path> path =  quinticer.quinticPlan();

    Scene test_scene;
    test_scene.X_C_map = curve_c_x;
    test_scene.Y_C_map = curve_c_y;
    test_scene.X_L_map = direct_x_l;
    test_scene.Y_L_map = direct_y_l;
    test_scene.X_M_map = direct_x_m;
    test_scene.Y_M_map = direct_y_m;
    test_scene.X_R_map = curve_x_r;
    test_scene.Y_R_map = curve_y_r;


    for (std::vector<Path>::iterator it = path.begin(); it!= path.end(); ++it) {
        X.push_back(it->xy.m_X);
        Y.push_back(it->xy.m_Y);
        yaw.push_back(it->yaw_des);
    }   

    // plt::figure(1);

    plt::plot(X,Y);
    plt::plot(X_L, Y_L, "k-");
    plt::plot(X_R,Y_R, "k-");
    plt::plot(X_M, Y_M, "k-");
    plt::scatter(C_X, C_Y, 50);
    // plt::show();


    std::string param = "/home/syn/毕业设计/路径规划/Project/data/vehicle_param.json";
    VehicleDynamicsModel model(0.01, 2, param);

    Eigen::Matrix4d Q;
    Eigen::Matrix<double, 1, 1> R;
    Q(0, 0) = 45;
    Q(1, 1) = 1;
    Q(2, 2) = 1;
    Q(3, 3) = 1;
    R << 55;

    LQRcontrol controller(model,Q,R );
    // PID pidcontroler(1.26836,5.37114e-07,1.26836);

    controller.setPath(path);

    std::vector<double> sim_x;
    std::vector<double> sim_y;

    // Plotting plotter;

    VehicleState v_state_s(0, 6.875, 0, 2);
    VehicleState v_state_e(20, 10.625, 0, 2);

    VehicleState state = v_state_s;

    // double t = 0;
    // double sim_t = 5; //s   仿真时间

    // while (t < sim_t) {
    //     std::array<double, 3> controlcommand = controller.runLQR(state);
    //     state.V = state.V;
    //     state.X += state.V * ts;
    //     state.yaw = controlcommand[0];
    //     state.Y += state.X * state.yaw;
        
    //     sim_x.push_back(state.X);
    //     sim_y.push_back(state.Y);
    //     t+=ts;
    // };

    LQRSpeed lqrspeed(q,r,state.V, 2);
    


    plt::plot(sim_x,sim_y);
    plt::show();

    return 0;
}