#include "plotting.h"

// 静态变量类外初始化
const double Plotting::car_L =car_length;
const double Plotting::car_W = CarWidth;
const double Plotting::rear_axle = 0.9;  //m 后悬

Plotting::Plotting() {
    // this->last_car_Location.m_X=0;
    // this->last_car_Location.m_Y=0;
}


/**
 * 输入： 当前车辆在全局坐标系下的位置，车辆的横摆角-车辆坐标系与全局坐标系的夹角
 * 输出： 无
 * 作用： 实时的在全局坐标系中把车画出来（矩形）
**/
void Plotting::plotCar (const XY &carlocation ,const double &phi ) const {
    XY corner_1;
    XY corner_2;
    XY corner_3;
    XY corner_4;

    corner_1.m_Y = -static_cast<double>(CarWidth/2.0);
    corner_1.m_X = static_cast<double>(car_length - rear_axle);

    corner_2.m_Y = static_cast<double>(CarWidth/2.0);
    corner_2.m_X = static_cast<double>(car_length - rear_axle);

    corner_3.m_Y = static_cast<double>(CarWidth/2.0);
    corner_3.m_X = -rear_axle;

    corner_4.m_Y = -static_cast<double>(CarWidth/2.0);
    corner_4.m_X = -rear_axle;

    std::vector<double> corner_x;
    std::vector<double> corner_y;

    corner_x.push_back( cartoglobal_coordinate(corner_1, carlocation, phi).m_X );
    corner_x.push_back( cartoglobal_coordinate(corner_2, carlocation, phi).m_X );
    corner_x.push_back( cartoglobal_coordinate(corner_3, carlocation, phi).m_X );
    corner_x.push_back( cartoglobal_coordinate(corner_4, carlocation, phi).m_X );
    corner_x.push_back( cartoglobal_coordinate(corner_1, carlocation, phi).m_X );

    corner_y.push_back( cartoglobal_coordinate(corner_1, carlocation, phi).m_Y );
    corner_y.push_back( cartoglobal_coordinate(corner_2, carlocation, phi).m_Y );
    corner_y.push_back( cartoglobal_coordinate(corner_3, carlocation, phi).m_Y );
    corner_y.push_back( cartoglobal_coordinate(corner_4, carlocation, phi).m_Y );
    corner_y.push_back( cartoglobal_coordinate(corner_1, carlocation, phi).m_Y );

    plt::plot(corner_x, corner_y, "k-");
};

/**
 * 输入： 地图文件的路径
 * 输出： 无
 * 作用： 画出地图场景
**/
void Plotting::plotRun() {
    // std::string  roadmapfile1 = "/home/syn/毕业设计/路径规划/project/roadmap/direct_roadmap.json";
    // // 加载地图场景文件
    // std::ifstream iRoad_Map(roadmapfile);
    // json json_RoadMap;
    // iRoad_Map >> json_RoadMap;

    // // 左侧道路 场景数据
    // std::vector<double> x_l_vec = json_RoadMap["LaneAndConeCoordinate"][0]["X_l"];
    // std::vector<double> y_l_vec = json_RoadMap["LaneAndConeCoordinate"][1]["Y_l"];
    // // 中间道路 场景数据
    // std::vector<double> x_m_vec = json_RoadMap["LaneAndConeCoordinate"][2]["X_m"];
    // std::vector<double> y_m_vec = json_RoadMap["LaneAndConeCoordinate"][3]["Y_m"];
    // // 右侧道路 场景数据
    // std::vector<double> x_r_vec = json_RoadMap["LaneAndConeCoordinate"][4]["X_r"];
    // std::vector<double> y_r_vec = json_RoadMap["LaneAndConeCoordinate"][5]["Y_r"];
    // // 锥桶 场景数据
    // std::vector<double> x_c_vec = json_RoadMap["LaneAndConeCoordinate"][6]["X_c"];
    // std::vector<double> y_c_vec = json_RoadMap["LaneAndConeCoordinate"][7]["Y_c"];

    // plt::figure(1);
    // plt::plot(x_l_vec, y_l_vec, "k-");
    // plt::plot(x_m_vec, y_m_vec, "r--");
    // plt::plot(x_r_vec, y_r_vec, "k-");
    // plt::scatter(x_c_vec, y_c_vec, 3);

    // plt::axis("equal");
    // plt::xlabel("X[m]");
    // plt::ylabel("Y[m]");
    // plt::show();
};

/**
 * 输入： 车辆实时规划出的轨迹点（从车辆坐标系转换到全局坐标系下）， 车辆在全局坐标系中的位置，车辆的横摆角
 * 输出： 无
 * 作用： 画出实时的仿真过程-轨迹以及车辆的移动
**/
void Plotting::plotSim( std::vector<XY> &trajectory, const Scene &scene, XY &carlocation ,const double &phi) {

    std::vector<double> plot_x;
    std::vector<double> plot_y;

    std::vector<double> plot_X_L_map(scene.X_L_map.data(), scene.X_L_map.data()+scene.X_L_map.size());
    std::vector<double> plot_Y_L_map(scene.Y_L_map.data(), scene.Y_L_map.data()+scene.Y_L_map.size());
    std::vector<double> plot_X_M_map(scene.X_M_map.data(), scene.X_M_map.data()+scene.X_M_map.size());
    std::vector<double> plot_Y_M_map(scene.Y_M_map.data(), scene.Y_M_map.data()+scene.Y_M_map.size());
    std::vector<double> plot_X_R_map(scene.X_R_map.data(), scene.X_R_map.data()+scene.X_R_map.size());
    std::vector<double> plot_Y_R_map(scene.Y_R_map.data(), scene.Y_R_map.data()+scene.Y_R_map.size());
    std::vector<double> plot_X_C_map(scene.X_C_map.data(), scene.X_C_map.data()+scene.X_C_map.size());
    std::vector<double> plot_Y_C_map(scene.Y_C_map.data(), scene.Y_C_map.data()+scene.Y_C_map.size());
    
    XY last_car_Location;
    last_car_Location.m_X = 0;
    last_car_Location.m_Y = 0;

    // if (this->last_car_Location != carlocation) 
    
    {
        // this->last_car_Location = carlocation;
        // plt::show();
        
        for (std::vector<XY>::iterator it = trajectory.begin();
            it!= trajectory.end();
            ++it) {

            plot_x.push_back(it->m_X);
            plot_y.push_back(it->m_Y);
        }

        // std::cout<< "plot map on "<<std::endl;
        plt::figure(1);
        plt::clf();
        plt::plot(plot_X_L_map, plot_Y_L_map, "r-");
        plt::plot(plot_X_M_map, plot_Y_M_map, "k--");
        plt::plot(plot_X_R_map, plot_Y_R_map, "r-");
        plt::scatter(plot_X_C_map,plot_Y_C_map, 5);
        plotCar(carlocation, phi);
        plt::plot(plot_x, plot_y);
        plt::axis("equal");
        // plt::xlim(0,220);
        // plt::ylim(0,15);
        // std::cout<< "plot track on "<<std::endl;

        plt::pause(0.01);
    }
    // plt::show();
}; 


/**
 * 输入： 德劳内三角剖分后的 边的vector
 * 输出： 无
 * 作用： 画出实时的德劳内三角剖分, 以及搜索出的路径
**/
void Plotting::plotDelaunay_planning( std::vector< std::pair<Point, unsigned> > points, 
                                        std::vector<Edge>& edge_vec,  
                                        std::vector<DPoint>& mid_point, 
                                        std::vector<Path> path) {

    //==============================================================================//
    // -------------------------德劳内三角剖分可视化-----------------------------------//
    for (std::vector<Edge>::iterator it = edge_vec.begin(); it!= edge_vec.end(); ++it) {
        double x1 ;
        double y1 ;
        double x2;
        double y2;

        for (std::vector< std::pair<Point, unsigned> > ::iterator it1 = points.begin();  it1!= points.end(); ++it1) {
            if (it->first == it1->second) {
                x1 = it1->first.x();
                y1 = it1->first.y();
            }
            else if (it->last == it1->second) {
                x2 = it1->first.x();
                y2 = it1->first.y();
            }
        }

        std::vector<double> X;
        std::vector<double> Y;
        X.push_back(x1);
        X.push_back(x2);
        Y.push_back(y1);
        Y.push_back(y2);
        // plt::named_plot("cone", );
        plt::named_plot("cone",X,Y,"gD--");
        // plt::plot(X,Y, "g--");
        // plt::legend(len);
// {{"color", "slategray"}, {"marker","D"},{"linestyle", "--"}}
    }

    //=============================================================================//
    //-----------------------------中点可视化---------------------------------------//
    std::vector<double> mid_point_X;
    std::vector<double> mid_point_Y;

    for (std::vector<DPoint>::iterator it = mid_point.begin();
            it != mid_point.end();
                ++it) {
        mid_point_X.push_back(it->m_X);
        mid_point_Y.push_back(it->m_Y);
    }
    plt::named_plot("Midpoint",mid_point_X, mid_point_Y, "ro");

    //=============================================================================//
    //-----------------------------路径可视化---------------------------------------//

    std::vector<double> rough_path_X;
    std::vector<double> rough_path_Y;

    for (std::vector<Path>::iterator it = path.begin();
            it != path.end();
                ++it) {
        rough_path_X.push_back(it->xy.m_X);
        rough_path_Y.push_back(it->xy.m_Y);
    }
    // {{"color", "chocolate"}, {"linestyle", "-"}}
    plt::named_plot("planned path",rough_path_X, rough_path_Y, "b-");
    // plt::save("./delaunay.svg");
    plt::xlabel("X(m)");
    plt::ylabel("Y(m)");
    // plt::legend();
    // plt::axis("equal");
    plt::ylim(4.5,10.5);
    plt::show();
};

/**
 * 输入： changjing leixing cheliangweizhi
 * 输出： 无
 * 作用： changjing plot
**/
void Plotting::plotScenario(ScenarioType& st, XY& car_Location) {
    std::vector<double> X_;
    std::vector<double> Y_;

    switch (st)
    {
    case LANECHANGE:
        X_.push_back(car_Location.m_X);
        Y_.push_back(1);
        plt::plot(X_, Y_);
        X_.clear();
        Y_.clear();
        break;
    
    case PURELANEKEEP:
        X_.push_back(car_Location.m_X);
        Y_.push_back(2);
        plt::plot(X_, Y_);
        X_.clear();
        Y_.clear();
        break;

    case CONELANEKEEP:
        X_.push_back(car_Location.m_X);
        Y_.push_back(3);
        plt::plot(X_, Y_);
        X_.clear();
        Y_.clear();
        break;

    case PURECONEKEEP:
        X_.push_back(car_Location.m_X);
        Y_.push_back(4);
        plt::plot(X_, Y_);
        X_.clear();
        Y_.clear();
        break;
    default:
        break;
    }

    //TODO  juece tu
};