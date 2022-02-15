/**
 *  根据 DriveEnv 的1周围环境信息 决策当前所处场景
 * 
**/
#ifndef _SCENARIO_MANAGER_
#define _SCENARIO_MANAGER_

#include <vector>

#include "drive_enviroment.h"


class ScenarioFSM
{
private:
    DriveEnv drive_env;
    // ScenarioType Default_Scenario_Type = PURELANEKEEP;
    ScenarioType Current_Scenario_Type = PURECONEKEEP;
    // 场景状态
    ScenarioStatus LaneChange_status = STATUS_UNKONW;

private:
    //TODO 考虑周围车辆状态 设计安全换道距离 决定是否可以换道
    void judgeScenarioType();

public:
    explicit ScenarioFSM(DriveEnv &driveEnv);

    void updateScenario( DriveEnv &drivenv);

    const ScenarioType& getScenarioType();
    
    // 类内注册一个 回调函数 用以更改 status  
    // TODO 回调函数
    void ChangeLaneChangeStatus();  // 传入 islanechangeover的函数指针？
    void registerIsLaneChangeOverCallBack();
};









#endif /*_SCENARIO_MANAGER_*/