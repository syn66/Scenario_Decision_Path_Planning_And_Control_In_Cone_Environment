#include <iostream>

#include "drive_enviroment.cpp"

int main(){

    std::vector<XY> cone1;
    std::vector<XY> cone2;
    std::vector<LaneFunc> lanfunc1;

    LaneFunc Lane1;
    Lane1.A_LaneFunc =0;
    Lane1.B_LaneFunc=0;
    Lane1.C_LaneFunc=0;
    Lane1.D_LaneFunc= -1.725;
    LaneFunc Lane2;
    Lane2.A_LaneFunc =0;
    Lane2.B_LaneFunc=0;
    Lane2.C_LaneFunc=0;
    Lane2.D_LaneFunc= 1.725;

    
    // 锥桶在车道线上
    XY xy1;
    xy1.m_X=1.0;
    xy1.m_Y=-1.725;

    XY xy2;
    xy2.m_X=3.0;
    xy2.m_Y=-1.725;
    XY xy3;
    xy3.m_X=4.0;
    xy3.m_Y=-1.725;
    XY xy4;
    xy4.m_X=5.0;
    xy4.m_Y=-1.725;
    
    
    // 锥桶在车道线内
    XY xy01;
    xy01.m_X=2;
    xy01.m_Y=-1.4;
    XY xy02;
        xy02.m_X=3;
    xy02.m_Y=-0.75;
    XY xy03;
        xy03.m_X=4;
    xy03.m_Y=0.75;
    XY xy04;
        xy04.m_X=5;
    xy04.m_Y=1.4;


     // 锥桶在车道线上
    cone1.push_back(xy1);
    cone1.push_back(xy2);
    cone1.push_back(xy3);
    cone1.push_back(xy4);
     // 锥桶在车道线内
    cone2.push_back(xy01);
    cone2.push_back(xy02);
    cone2.push_back(xy03);
    cone2.push_back(xy04);

    std::vector<XY>  cone4;
    XY xy12;
    xy12.m_X =4;
    xy12.m_Y=0;
    cone4.push_back(xy12);

    // XY xy = cone1[0];
    // std::cout<< xy.m_X<< xy.m_Y<<std::endl;

    lanfunc1.push_back(Lane1);
    lanfunc1.push_back(Lane2);

    DriveEnv d1(cone1,lanfunc1);

    std::vector<XY> cone3;

    std::cout<< d1.getlanetype() <<std::endl;
    std::cout<< d1.getconside() <<std::endl;
    std::cout<< d1.getConeStatusByLane() << std::endl;
    std::cout<< d1.isLaneTyprChanged() << std::endl;
    std::cout<< d1.getDistofconeTocar() << std::endl;
    
    

    d1.updateEnv(cone2, lanfunc1);


    std::cout<< "_______update________" <<std::endl;
    std::cout<< d1.getlanetype() <<std::endl;
    std::cout<< d1.getconside() <<std::endl;
    std::cout<< d1.getConeStatusByLane() << std::endl;
    std::cout<< d1.isLaneTyprChanged() << std::endl;
    std::cout<< d1.getDistofconeTocar() << std::endl;


    d1.updateEnv(cone3, lanfunc1);


    std::cout<< "_______update________" <<std::endl;
    std::cout<< d1.getlanetype() <<std::endl;
    std::cout<< d1.getconside() <<std::endl;
    std::cout<< d1.getConeStatusByLane() << std::endl;
    std::cout<< d1.isLaneTyprChanged() << std::endl;
    std::cout<< d1.getDistofconeTocar() << std::endl;



    // DriveEnv d1();
    
    return 0;
}