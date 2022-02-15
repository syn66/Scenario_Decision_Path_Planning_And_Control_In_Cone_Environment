/***
 *  根据道路周围 环境 （锥桶坐标与车道线方程） 获取当前 道路类型 
***/

#ifndef _DRIVEENVIROMENT_
#define _DRIVEENVIROMENT_

#include <vector>
#include <algorithm>

#include "../config.h"

// 用以描述车辆周围环境
class DriveEnv
{
private:
    // 车辆坐标系下
    std::vector<XY> m_coneXY;
    std::vector<LaneFunc> m_LaneFunc;
    // 车辆左右车道线 方程索引
    LaneNumber nearestLane;

    LaneType CurrentLaneType;
    LaneType PreviousLaneType = PURELANE;
    // 距离车辆最近的锥桶距离 
    float DistOfConeToCar = 0;

    // 默认值 无锥桶
    bool isConeInLane;
    bool isConeOnLane;
    ConeSide conside = CONE_NULL;

    void findNearestlane();
    std::vector<float> findMinD2( std::vector<float>& sample_X, LaneFunc& lanefunc);

    void Init();
    void Ifconeonlane();
    void Ifconeinlane();
    
public:
    explicit DriveEnv( std::vector<XY> coneXY, std::vector<LaneFunc> lanefunc);
    
    void updateEnv(std::vector<XY>& Next_xy, std::vector<LaneFunc>& NextLaneFunc);

    // 获取当前 道路类型
    const LaneType& getlanetype();
    const ConeSide& getconside();

    const ConeStatueByLane& getConeStatusByLane();

    const bool isLaneTyprChanged();
    const float getDistofconeTocar() ;

    const std::vector<LaneFunc>& getNearlane();

    const int getNumofcone();
};


#endif /*_DRIVEENVIROMENT_*/