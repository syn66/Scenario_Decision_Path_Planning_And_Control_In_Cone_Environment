#include "lanekeep.h"

int PureLaneKeep::Sample_num_X = 30; 
int PureLaneKeep::Sample_step_X = 1;  //m


PureLaneKeep::PureLaneKeep( std::vector<LaneFunc> nearlane):
                            near_Lane(nearlane) {};

// 提取中线 规划
std::vector<XY> PureLaneKeep::plan() {

    std::vector<double> sample_x; // 采样点

    std::vector<XY> middle_point_XY;  //中点数组

    for (int i=0; i<this->Sample_num_X; i+=this->Sample_step_X) {
        sample_x.push_back(i);
    }
    // TODO 回调函数 如果车道的采样点小于 阈值 并且 锥桶坐标不为空 就回调 切换场景为纯锥桶
    for (std::vector<double>::iterator it=sample_x.begin(); it!=sample_x.end(); ++it) {
        double sample_L_y = solveFunction(near_Lane[0], *it);
        double sample_R_y = solveFunction(near_Lane[1], *it);

        double middle_x = *it;
        double middle_y = (sample_L_y+sample_R_y)/2.0 ;

        XY middle_xy;
        middle_xy.m_X = middle_x;
        middle_xy.m_Y = middle_y;

        middle_point_XY.push_back(middle_xy);
    }
    return middle_point_XY;
};


ConeLaneKeep::ConeLaneKeep( std::vector<LaneFunc> nearlane, 
                            std::vector<XY> cone_XY, 
                            ConeSide coneside):
                            near_Lane(nearlane),
                            cone_point(cone_XY),
                            current_cone_side(coneside) {};


std::vector<Path> ConeLaneKeep::plan() {

    std::vector<XY> traj_XY;
    for (std::vector<XY>::iterator it=this->cone_point.begin(); it!=this->cone_point.end(); ++it) {
        XY traj_xy;
        double lanepoint_y;
        
        switch (this->current_cone_side)
        {
        case CONE_L:
            lanepoint_y = solveFunction(near_Lane[1], it->m_X);
            traj_xy.m_X = it->m_X;
            traj_xy.m_Y = (it->m_Y + lanepoint_y)/2.0;
            traj_XY.push_back(traj_xy);
            break;

        case CONE_R:
            lanepoint_y = solveFunction(near_Lane[0], it->m_X);
            traj_xy.m_X = it->m_X;
            traj_xy.m_Y = (it->m_Y + lanepoint_y)/2.0;
            traj_XY.push_back(traj_xy);
            break;
            
        default:
            break;
        }
    }
    // std::cout<< traj_XY.size() << '\n';
    Bezier smooother(traj_XY);

    // std::vector<Path> raj_xy;
    
    // for(std::vector<XY>::iterator it = traj_XY.begin(); it!=traj_XY.end(); ++it) {
    //     Path temp;
    //     temp.xy.m_X = it->m_X;
    //     temp.xy.m_Y = it->m_Y;
    //     raj_xy.push_back(temp);
    // }

    std::vector<Path> smoothPth = smooother.Smooth();
    smooother.showpoints();
    
    return smoothPth;

    // return raj_xy;
};