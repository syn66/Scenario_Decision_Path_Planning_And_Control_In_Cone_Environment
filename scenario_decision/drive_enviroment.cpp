
#include "drive_enviroment.h"
#include "../utils.h"

/**
 * 输入： 锥桶坐标，车道线方程
 * 输出： 无
 * 作用： 构造 当前车辆周围环境对象
**/
DriveEnv::DriveEnv(std::vector<XY> coneXY, std::vector<LaneFunc> lanefunc) {

    this->m_coneXY = coneXY;
    this->m_LaneFunc = lanefunc;
    Init();
    // std::cout<<"构造函数"<<std::endl;
};

/**
 * 输入： 无
 * 输出： 无
 * 作用：  初始化对象
**/
void DriveEnv::Init() {

    // 判断车道类型 并 初始化 currentlanetype
    if (!this->m_coneXY.empty() &&
        !this->m_LaneFunc.empty() ) {

            this->CurrentLaneType = MIXEDLANE;
            this->DistOfConeToCar = findnearestconeX(this->m_coneXY);

    }
    else if (this->m_LaneFunc.empty() && 
            !this->m_coneXY.empty()) {

            this->CurrentLaneType = PURECONE;
            this->DistOfConeToCar = findnearestconeX(this->m_coneXY);

    }
    else if (this->m_coneXY.empty() && 
            !this->m_LaneFunc.empty()) {
        this->CurrentLaneType = PURELANE;
        this->DistOfConeToCar = 0;
    }

    // 初始化 锥桶 具体场景
    findNearestlane();
    Ifconeinlane();
    Ifconeonlane();
};


/**
 * 找到 车辆左右两侧的车道编号
 * 首先 依据 车道线方程中的 D_LaneFunc 系数 对数组 m_lanefunc进行排序 从小到大
 * 然后将 车道线方程数组 依据 D_LaneFunc 分为大于0 和小于 0 ；
 * 然后 size 函数 获得 小于 0 的数组大小 temp， 那么在车辆 左侧的车道索引为 temp-1， 右侧的为temp
**/
/**
 * 输入： 无
 * 输出： 无
 * 作用： 找到车辆坐标系的车道线中 在车辆两侧的车道线
**/
void DriveEnv::findNearestlane()
{
    // 存放 D_LaneFunc 大于0 和 小于0 的 车道线方程
    std::vector<LaneFunc> Lminzero;
    std::vector<LaneFunc> Lmaxzero;
    std::sort(this->m_LaneFunc.begin(), this->m_LaneFunc.end(), CompareByD);

    for (std::vector<LaneFunc>::iterator it = this->m_LaneFunc.begin(); it != this->m_LaneFunc.end(); ++it)
    {
        if ((*it).D_LaneFunc < 0)
        {
            Lminzero.push_back(*it);
        }
        else if ((*it).D_LaneFunc > 0)
        {
            Lmaxzero.push_back(*it);
        }
    }
    int temp = Lminzero.size();
    
    this->nearestLane.LaneL = temp - 1;
    this->nearestLane.LaneR = temp;
};


/**
 * 输入： 车道采样点，车道方程
 * 输出： vector数组
 * 作用： 通过采样点 找到每一个锥桶坐标到车道线的最小距离 并存进到一个数组里并返回
**/
std::vector<float> DriveEnv::findMinD2( std::vector<float>& sample_X, LaneFunc& lanefunc) {

    std::vector<float> mind2;
    mind2.reserve(12);

    for (std::vector<XY>::iterator it =this->m_coneXY.begin(); it!= this->m_coneXY.end(); ++it) {
        {   
            //存放所有采样点 到 锥桶的距离
            std::vector<float> D2;
            // 计算 采样点 y值
            for (std::vector<float>::iterator it1= sample_X.begin(); it1!= sample_X.end(); ++it1) {
                float fx = solveFunction(lanefunc, *it1);
                float d2 = Calc_Dist( (*it).m_X ,*it1 ,(*it).m_Y, fx );
                D2.push_back(d2);
            }
            // 将 最小距离存储进 mind2
            mind2.push_back( *std::min_element( D2.begin(), D2.end() ) );
        }
    }
    return mind2;
};

/**
 * 输入： 下一时刻的锥桶坐标 与 车道线方程
 * 输出： 无
 * 作用： 更新当前对象的状态
**/
void DriveEnv::updateEnv(std::vector<XY>& Next_xy, std::vector<LaneFunc>& NextLaneFunc) {

    this->m_LaneFunc = NextLaneFunc;
    this->m_coneXY = Next_xy;
    this->PreviousLaneType = this->CurrentLaneType;
    Init();
};

/**
 * 输入： 无
 * 输出： 车道类型-常量
 * 作用： 接口-获取当前车道类型
**/
const LaneType& DriveEnv::getlanetype() {
    return this->CurrentLaneType;
};

/**
 * 输入： 无
 * 输出： 锥桶在哪一侧-常量
 * 作用： 接口-返回当前锥桶在那一侧车道
**/
const ConeSide& DriveEnv::getconside(){
    return this->conside;
};

/**
 * 输入： 无
 * 输出： 无
 * 作用： 判断锥桶是否在车道线内
**/
void DriveEnv::Ifconeinlane()
{
    if (this->CurrentLaneType == MIXEDLANE) {

        std::vector<float> sample_x_R;
        std::vector<float> sample_x_L;
        
        sample_x_R.reserve(310);
        sample_x_L.reserve(310);

        // lane采样范围 0-30m
        for (float i=0; i< DetectRange; i+=0.1) {
            sample_x_R.push_back(i);
            sample_x_L.push_back(i);
        }

        std::vector<float> mind2L = findMinD2(sample_x_L, this->m_LaneFunc[this->nearestLane.LaneL]);
        std::vector<float> mind2R = findMinD2(sample_x_R, this->m_LaneFunc[this->nearestLane.LaneR]);
        // 距离 判断 锥桶是否在 车道内

        std::cout<< *std::max_element(mind2R.begin(), mind2R.end()) <<'\n';
        std::cout<< *std::max_element(mind2L.begin(), mind2L.end()) <<'\n';

        if ( *std::max_element(mind2R.begin(), mind2R.end()) < LaneWidth && 
                *std::max_element(mind2L.begin(), mind2L.end()) < LaneWidth) {

            this->isConeInLane = true;
        }
        else {

            this->isConeInLane = false;
        }
    }
    else {
        this->isConeInLane = false;
    }
};

/**
 * 输入： 无
 * 输出： 无
 * 作用： 判断锥桶是否在车道线上
**/
void DriveEnv::Ifconeonlane()
{
    if (this->CurrentLaneType == MIXEDLANE) {
        std::vector<float> fxL;
        std::vector<float> fxR;
        std::vector<float> fx;
        double mean_err_L=0;
        double mean_err_R=0;
        //  预留空间 减少动态扩展次数  预留空间根据 感知能力确定
        fxL.reserve(12);
        fxR.reserve(12);
        fx.reserve(12);

        for (std::vector<XY>::iterator it = this->m_coneXY.begin(); it != this->m_coneXY.end(); ++it) {
            // 将 锥桶 x坐标 对应的 车道线方程上的 y值存在数组
            fx.push_back((*it).m_Y);
            fxL.push_back(solveFunction(this->m_LaneFunc[this->nearestLane.LaneL], (*it).m_X));
            fxR.push_back(solveFunction(this->m_LaneFunc[this->nearestLane.LaneR], (*it).m_X));
        }
        {
            std::vector<double> errol;
            std::vector<double> error;
            
            errol.reserve(12);
            error.reserve(12);
            
            for (std::vector<float>::iterator it = fx.begin(); it != fx.end(); ++it) {

                auto index = std::distance(fx.begin(), it);
                double erroL = fabs(fxL[index] - (*it));
                double erroR = fabs(fxR[index] - (*it));
                // 以 锥桶x所得到的车道线fx 与 锥桶的fx 的 误差来判断 锥桶是否在车道线上   0.6 为锥桶 在车道线上允许的偏差
                errol.push_back(erroL);
                error.push_back(erroR);
            }
            // 求方差
            mean_err_L = variance(errol);
            mean_err_R = variance(error);
        }
        if (mean_err_R < ConeErro) {
            this->conside = CONE_R;
            this->isConeOnLane = true;
        }
        else if( mean_err_L < ConeErro) {
            this->conside = CONE_L;
            this->isConeOnLane = true;
        }
        else  {
            this->conside = CONE_NULL;
            this->isConeOnLane = false;
        }
    }
    else {
        this->conside = CONE_NULL;
        this->isConeOnLane = false;
    }
};

/**
 * 输入： 无
 * 输出： 锥桶相对与车道的状态-常量
 * 作用： 接口-返回当前锥桶相对于车道的状态（车道内，上，未知）
**/
const ConeStatueByLane& DriveEnv::getConeStatusByLane() {
    
    static ConeStatueByLane csbl = CONESTATUS_NULL;

    if (this->CurrentLaneType ==MIXEDLANE) {
        if ( this->isConeInLane ) {
            csbl = INLANE;
        }
        else if (this->isConeOnLane) {
            csbl = ONLANE;
        }
    }
    else if (this->CurrentLaneType == PURECONE || this->CurrentLaneType == PURELANE) {
        csbl = CONESTATUS_NULL;
    }
    return csbl;
};

/**
 * 输入： 无
 * 输出： 布尔值
 * 作用： 判断车道类型是否发生改变 
**/
const bool DriveEnv::isLaneTyprChanged() {
    bool islanetypechanged = false;
    if (this->CurrentLaneType != this->PreviousLaneType) {
        islanetypechanged = true;
    }
    return islanetypechanged;
};

const float DriveEnv::getDistofconeTocar() {
    return this->DistOfConeToCar;
};

/**
 * 输入： 无
 * 输出： 车辆两侧车道线方程数组
 * 作用：   接口-返回车辆两侧车道线方程
**/
const std::vector<LaneFunc>& DriveEnv::getNearlane() {
    static std::vector<LaneFunc> nearlane;
    nearlane.push_back( this->m_LaneFunc[this->nearestLane.LaneL]);
    nearlane.push_back( this->m_LaneFunc[this->nearestLane.LaneR]);
    return nearlane;
};  

/**
 * 输入： 无
 * 输出： 锥桶的个数
 * 作用：   接口-返回锥桶的个数
**/
const int DriveEnv::getNumofcone() {
    return this->m_coneXY.size();
};
