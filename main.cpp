
#include "Plotting/plotting.h"
#include "roadmap/load_map.h"
#include "scenario_decision/drive_enviroment.h"
#include "scenario_decision/scenario_manager.h"

// std::vector<XY>& getTraj(std::vector<XY> &xy_track) {
//     static std::vector<XY> traj;
//     for (std::vector<XY>::iterator it= xy_track.begin(); it!=xy_track.end(); ++it) {
//         traj.assign(it, xy_track.end());

//     }
//     return traj;
// };

// XY& getCarlocation(std::vector<XY> &xy_track) {
//     static XY car_Location;
//     for (std::vector<XY>::iterator it= xy_track.begin(); it!=xy_track.end(); ++it) {
//         car_Location.m_X = it->m_X;
//         car_Location.m_Y = it->m_Y;

//     }
//     return car_Location;
// };
namespace plt = matplotlibcpp ;

int main(){

    std::string mapfile = "/home/syn/毕业设计/路径规划/Project/data/direct_roadmap.json";
    std::string trackfile = "/home/syn/毕业设计/路径规划/Project/data/direct_track_lanechange.json";

    std::ifstream ifs_track(trackfile);
    
    json json_track;
    ifs_track >> json_track;

    std::vector<double> x_track = json_track["track"][0]["X_t"];
    std::vector<double> y_track = json_track["track"][1]["Y_T"];

    Eigen::VectorXd X_Track = Eigen::Map<Eigen::VectorXd>(x_track.data(), x_track.size());
    Eigen::VectorXd Y_Track = Eigen::Map<Eigen::VectorXd>(y_track.data(), y_track.size());

    // 实例化 加载地图对象
    LoadMap loadmap(mapfile);
    // 实例化车辆周围环境对象
    DriveEnv drive_env( loadmap.getCurrentConeCoor(), loadmap.getCurrentlaneFunc() );
    // 实例化 场景决策类
    ScenarioFSM scenario_decider(drive_env);
    // 
    std::vector<XY> track;
    for (int i =0; i<X_Track.size(); ++i) {
        XY track_XY;
        track_XY.m_X = X_Track[i];
        track_XY.m_Y = Y_Track[i];

        track.push_back(track_XY);
    };
    // dangqian guiji yu cheliagweizhi
    std::vector<XY> current_trak;
    XY current_car_location;
    current_car_location.m_X=0;
    current_car_location.m_Y=0;
    double phi = 0;

    Plotting  plotter;

    std::vector<double> sim_X;
    std::vector<double> sim_Y;

    for (std::vector<XY>::iterator it = track.begin(); it!=track.end();++it) {

        current_car_location = *it;
        current_trak.assign(it, track.end());

        // if (  35< it->m_X <45 || 95< it->m_X <105)  {
        //     phi = 45;
        // }
        // else {
        //     phi = 0;
        // }

        loadmap.updateCurrentMap( *it, phi);
        drive_env.updateEnv(loadmap.getCurrentConeCoor(), loadmap.getCurrentlaneFunc());
        scenario_decider.updateScenario(drive_env);

        // std::cout<<"锥桶状态   " <<drive_env.getConeStatusByLane()<<'\n';
        // std::cout<<"车道类型   " <<drive_env.getlanetype() <<'\n';
        std::cout<< "场景类型 " << scenario_decider.getScenarioType()<<'\n';
        std::cout<< "轨迹点     "<<current_trak[0].m_X<< '\n';
        // std::cout<< "车辆位置   "<<current_car_location.m_X<< '\n';
        std::cout<< "cone from 车辆位置   "<<drive_env.getDistofconeTocar()<< '\n';

        switch (scenario_decider.getScenarioType())
        {
        case LANECHANGE:
            sim_X.push_back(it->m_X);
            sim_Y.push_back(1);
            break;
        
        case PURELANEKEEP:
            sim_X.push_back(it->m_X);
            sim_Y.push_back(2);
            break;

        case CONELANEKEEP:
            sim_X.push_back(it->m_X);
            sim_Y.push_back(3);
            break;

        case PURECONEKEEP:
            sim_X.push_back(it->m_X);
            sim_Y.push_back(4);
            break;

        default:
            break;
        }
        // std::cout<< "chedao fangcheng daxiao   "<< loadmap.getCurrentlaneFunc().size() <<'\n';

        // plotter.plotSim(current_trak, loadmap.getMap(), current_car_location, phi);

    };

    Scene scene = loadmap.getMap();

    std::vector<double> plot_X_L_map(scene.X_L_map.data(), scene.X_L_map.data()+scene.X_L_map.size());
    std::vector<double> plot_Y_L_map(scene.Y_L_map.data(), scene.Y_L_map.data()+scene.Y_L_map.size());
    std::vector<double> plot_X_M_map(scene.X_M_map.data(), scene.X_M_map.data()+scene.X_M_map.size());
    std::vector<double> plot_Y_M_map(scene.Y_M_map.data(), scene.Y_M_map.data()+scene.Y_M_map.size());
    std::vector<double> plot_X_R_map(scene.X_R_map.data(), scene.X_R_map.data()+scene.X_R_map.size());
    std::vector<double> plot_Y_R_map(scene.Y_R_map.data(), scene.Y_R_map.data()+scene.Y_R_map.size());
    std::vector<double> plot_X_C_map(scene.X_C_map.data(), scene.X_C_map.data()+scene.X_C_map.size());
    std::vector<double> plot_Y_C_map(scene.Y_C_map.data(), scene.Y_C_map.data()+scene.Y_C_map.size());


    plt::subplot(2,1,1);
    plt::plot(sim_X,sim_Y,"g-");
    // plt::axis("equal");
    plt::ylim(0,5);
    plt::xlim(0,188);
    // plt::xlabel("x/m");
    plt::ylabel("decision result");

    plt::subplot(2,1,2);
    plt::plot(plot_X_L_map, plot_Y_L_map,"k-");
    plt::plot(plot_X_M_map, plot_Y_M_map,"k--");
    plt::plot(plot_X_R_map, plot_Y_R_map,"k-");
    plt::scatter(plot_X_C_map, plot_Y_C_map, 20);
        // plt::ylim(0,5);
    plt::xlim(0,188);
    plt::xlabel("x/m");
    plt::ylabel("y/m");

    plt::save("juece.svg");
    plt::show();
    // plotter.plotSim(current_trak, loadmap.getMap(), current_car_location, phi);

    // plotter.plotSim(track, loadmap.getMap(), current_car_location, phi);

    

    return 0;

}


