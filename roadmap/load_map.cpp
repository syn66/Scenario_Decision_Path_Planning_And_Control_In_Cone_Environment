
#include "load_map.h"


const int LoadMap::PointHorizon = 60;

/**
 * 函数名： 构造函数
 * 输入： 地图文件路径
 * 输出：   无
 * 作用：   构造一个loadmap对象
**/
LoadMap::LoadMap( std::string road_map_file) {
    
    init(road_map_file);

    // TODO
    assignthorizon();
    polyfitLanefuc();
};

/**
 * 函数名： 初始化
 * 输入： 地图文件路径
 * 输出：   无
 * 作用：  对类的成员变量X_L_vec，Y_L_vec 进行初始化
**/
void LoadMap::init( std::string road_map_file) {
    // std::cout<<road_map_file<<std::endl;
    // 加载地图场景文件
    std::ifstream iRoadMap(road_map_file);
    json jsonRoadMap;
    iRoadMap >> jsonRoadMap;
    // 左侧道路 场景数据
    std::vector<double> x_l_vec = jsonRoadMap["LaneAndConeCoordinate"][0]["X_l"];
    this->X_L_vec = Eigen::Map<Eigen::VectorXd>(x_l_vec.data(), x_l_vec.size());

    std::vector<double> y_l_vec = jsonRoadMap["LaneAndConeCoordinate"][1]["Y_l"];
    this->Y_L_vec = Eigen::Map<Eigen::VectorXd>(y_l_vec.data(), y_l_vec.size());
    // 中间道路 场景数据
    std::vector<double> x_m_vec = jsonRoadMap["LaneAndConeCoordinate"][2]["X_m"];
    this->X_M_vec = Eigen::Map<Eigen::VectorXd>(x_m_vec.data(), x_m_vec.size());

    std::vector<double> y_m_vec = jsonRoadMap["LaneAndConeCoordinate"][3]["Y_m"];
    this->Y_M_vec = Eigen::Map<Eigen::VectorXd>(y_m_vec.data(), y_m_vec.size());
    // 右侧道路 场景数据
    std::vector<double> x_r_vec = jsonRoadMap["LaneAndConeCoordinate"][4]["X_r"];
    this->X_R_vec = Eigen::Map<Eigen::VectorXd>(x_r_vec.data(), x_r_vec.size());

    std::vector<double> y_r_vec = jsonRoadMap["LaneAndConeCoordinate"][5]["Y_r"];
    this->Y_R_vec = Eigen::Map<Eigen::VectorXd>(y_r_vec.data(), y_r_vec.size());
    // 锥桶 场景数据
    std::vector<double> x_c_vec = jsonRoadMap["LaneAndConeCoordinate"][6]["X_c"];
    this->X_C_vec = Eigen::Map<Eigen::VectorXd>(x_c_vec.data(), x_c_vec.size());

    std::vector<double> y_c_vec = jsonRoadMap["LaneAndConeCoordinate"][7]["Y_c"];
    this->Y_C_vec = Eigen::Map<Eigen::VectorXd>(y_c_vec.data(), y_c_vec.size());

    XY car_init_location; //暂定 后面调参
    car_init_location.m_X = 0;
    car_init_location.m_Y = 10.625;
    double car_init_head_angle = 0 ; // 暂定 

    this->car_location = car_init_location;
    this->car_head_angle = car_init_head_angle;
};

/**
 * 输入： 类 成员变量
 * 输出：   无
 * 作用：  对类成员变量 x_L_horizon，Y_L_horizon等赋值 检测范围内的
**/
void LoadMap::assignthorizon() {
    // 确定当前车辆位置在 车道场景中的相似数据索引
    // this->index_L = indexLanePoint(this->X_L_vec, this->Y_L_vec, this->index_L,this->car_location);
    this->index = indexLanePoint(this->X_M_vec, this->index,this->car_location);
    // this->index_R = indexLanePoint(this->X_R_vec, this->Y_R_vec, this->index_R,this->car_location);
    this->Coor_C_Horizon.resize(0);
    // cone jiazai
    assignCone(this->X_C_vec, this->Y_C_vec, this->car_location, this->Coor_C_Horizon, this->car_head_angle);

    // if (Coor_C_Horizon.empty()) {
    //     std::cout<< "no detecte cone"<<'\n';
    // } 
    // else {
    //     std::cout<< Coor_C_Horizon[3].m_Y <<'\n';
    // }

    // panduan chedaoxian shifou jiazai wan
    if ( this->car_location.m_X  <= 159.5 ){
        // 循环索引 地图中点 并存放进coor_horizon  存疑
        indexAndassign(this->index, PointHorizon,
                        X_L_vec, Y_L_vec,
                        this->X_L_horzion, this->Y_L_horzion, 
                        this->car_location, this->car_head_angle);
        indexAndassign(this->index, PointHorizon,
                        X_M_vec, Y_M_vec,
                        this->X_M_horzion, this->Y_M_horzion, 
                        this->car_location, this->car_head_angle);
        indexAndassign(this->index, PointHorizon,
                        X_R_vec, Y_R_vec,
                        this->X_R_horzion, this->Y_R_horzion, 
                        this->car_location, this->car_head_angle);

        std::cout<< Y_L_horzion[0]<<'\n';
        std::cout<< Y_M_horzion[0]<<'\n';
        std::cout<< Y_R_horzion[0]<<'\n';
    }
    else {
        this->X_L_horzion.resize(0);
        this->Y_L_horzion.resize(0);

        this->X_M_horzion.resize(0);
        this->Y_M_horzion.resize(0);

        this->X_R_horzion.resize(0);
        this->Y_R_horzion.resize(0);
    }


};

/**
 * 输入： 类成员变量
 * 输出：  无
 * 作用： 根据Y_L_horizono，X_L_horizon等成员变量 拟合左中右三条车道线 将车道线系数存进 Lanefunc_L等 
**/
void LoadMap::polyfitLanefuc() {

    LaneFunc lanefunc_L;
    LaneFunc lanefunc_M;
    LaneFunc lanefunc_R;
    // ruguo youchedaoxian
    if (this->X_L_horzion.size() != 0 ) {

        auto coeff_L = polyfit(this->X_L_horzion, this->Y_L_horzion, 3);
        auto coeff_M = polyfit(this->X_M_horzion, this->Y_M_horzion, 3);
        auto coeff_R = polyfit(this->X_R_horzion, this->Y_R_horzion, 3);

        lanefunc_L = assingcoeff(coeff_L);
        lanefunc_M = assingcoeff(coeff_M);
        lanefunc_R = assingcoeff(coeff_R);

        this->PolyFitLaneFunc.push_back(lanefunc_L);
        this->PolyFitLaneFunc.push_back(lanefunc_M);
        this->PolyFitLaneFunc.push_back(lanefunc_R);

    }
    // meiyou chedoaxian
    else {
        this->PolyFitLaneFunc.resize(0);
    };
};

/**
 * 输入： 类成员变量
 * 输出：  车道线方程
 * 作用： 接口，获取当前车道线的车道线方程 
**/
std::vector<LaneFunc>& LoadMap::getCurrentlaneFunc() {
    return this->PolyFitLaneFunc;
};

/**
 * 输入： 类成员变量
 * 输出：  锥桶坐标
 * 作用： 接口，获取当前 锥桶坐标 
**/
std::vector<XY>& LoadMap::getCurrentConeCoor() {
    return this->Coor_C_Horizon;
};

/**
 * 输入： 类成员变量
 * 输出： 车辆在地图中的位置坐标
 * 作用： 接口，获取当前 车辆位置
**/
const XY& LoadMap::getcurrentCarlocation() {
    return this->car_location;
};

/**
 * 输入： 控制/规划模块返回的车辆位置， 车辆的横摆角
 * 输出： 无
 * 作用： 更新当前 x_L_horizon等成员变量
**/
void LoadMap::updateCurrentMap( XY& car_location_return, double &phi_return ) {
    if ( Calc_Dist(this->car_location.m_X, car_location_return.m_X, 
                    this->car_location.m_Y, car_location_return.m_Y) > mapUpdatestep ) {

        this->car_head_angle = phi_return;
        this->car_location = car_location_return;

        this->PolyFitLaneFunc.resize(0);
        assignthorizon();
        polyfitLanefuc();
    }
};

const Scene& LoadMap::getMap() {
    static Scene temp;
    temp.X_L_map = this->X_L_vec;
    temp.Y_L_map = this->Y_L_vec;
    temp.X_M_map = this->X_M_vec;
    temp.Y_M_map = this->Y_M_vec;
    temp.X_R_map = this->X_R_vec;
    temp.Y_R_map = this->Y_R_vec;
    temp.X_C_map = this->X_C_vec;
    temp.Y_C_map = this->Y_C_vec;

    return temp;
};