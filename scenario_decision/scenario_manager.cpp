
#include "scenario_manager.h"

// 构造函数
ScenarioFSM::ScenarioFSM(DriveEnv &driveEnv):
            drive_env(driveEnv) {   
    
    judgeScenarioType();
};

/**
 * 输入： 无
 * 输出： 无
 * 作用： 判断当前 场景
**/
void ScenarioFSM::judgeScenarioType() {

    switch ( this->drive_env.getlanetype() ) {
        case PURELANE:
            // 场景为车道保持  换道状态为未知
            this->Current_Scenario_Type = PURELANEKEEP;
            this->LaneChange_status = STATUS_UNKONW;
            break;

        case PURECONE:
            // 场景为锥桶车道保持  换道状态为未知
            this->Current_Scenario_Type = PURECONEKEEP;
            this->LaneChange_status = STATUS_UNKONW;
            break;

        case MIXEDLANE:
            if (this->drive_env.getConeStatusByLane() == INLANE) {
                // TODO 换道决策
                // 进入换道范围 场景进入 换道场景
                if (this->drive_env.getDistofconeTocar() < LaneChangeRange) {
                    // 场景为 换道  换道状态为 正在处理
                    this->Current_Scenario_Type = LANECHANGE;
                    this->LaneChange_status = STATUS_PROCESSING;
                }
                else {
                    // 场景为车道保持  换道状态为未知
                    this->Current_Scenario_Type = PURELANEKEEP;
                    this->LaneChange_status = STATUS_UNKONW;
                }
            }
            else if ( this->drive_env.getConeStatusByLane() == ONLANE) {
                // 场景为混合车道保持  换道状态为未知
                this->Current_Scenario_Type = CONELANEKEEP;
                this->LaneChange_status = STATUS_UNKONW;
                // 如果当前在车道线上的锥桶小于 3 就切换场景为纯车道保持
                if (this->drive_env.getNumofcone() < 3) {
                    this->Current_Scenario_Type = PURELANEKEEP;
                }
            }
            break;
    }
};

/**
 * 输入： 无
 * 输出： 场景类型
 * 作用： 接口-返回当前scenario类型
**/
const ScenarioType& ScenarioFSM::getScenarioType() {
    return this->Current_Scenario_Type;
};

/**
 * 输入： DriveEnv 对象
 * 输出： 无
 * 作用： 更新当前场景类型
**/
void ScenarioFSM::updateScenario( DriveEnv& drivenv) {

    this->drive_env = drivenv;
    switch (this->Current_Scenario_Type) {
        case LANECHANGE:
            if (this->LaneChange_status = STATUS_DONE) {
                judgeScenarioType();
            }
            break;
        default:
            judgeScenarioType();
    }
};

