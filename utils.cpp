#include "utils.h"

// 求解车道线方程
double solveFunction(LaneFunc &lanefunc, float x)
{
    float fx = lanefunc.A_LaneFunc * x * x * x + lanefunc.B_LaneFunc * x * x + lanefunc.C_LaneFunc * x + lanefunc.D_LaneFunc;
    return fx;
};

//依据 车道线方程中的 D_LaneFunc 系数 对数组 m_lanefunc进行排序 从小到大
bool CompareByD(LaneFunc &lanefunc1, LaneFunc &lanefunc2)
{
    return lanefunc1.D_LaneFunc < lanefunc2.D_LaneFunc;
};

// 依据 锥桶坐标的 x值-距离车辆的纵向距离 来 对 m_conexy 排序 从小到大
bool CompareByX(XY &xy1, XY &xy2) {
    return xy1.m_X < xy2.m_X;
};

// 计算两点间距离
float Calc_Dist(float x1, float x2, float y1, float y2) {
    return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
};

// 方差 variance
double variance(std::vector<double> vec) {
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    double mean = sum/vec.size();
    double accum = 0.0;
    std::for_each(vec.begin(), vec.end(), [&](const double d) {
        accum +=(d-mean)*(d-mean);
    });
    double var = sqrt( accum/(vec.size()-1) );
    return var;
};

// 找出最小值 通过 x
float findnearestconeX( std::vector<XY> xy) {
    std::sort(xy.begin(), xy.end(), CompareByX);
    return xy[0].m_X;
};

// 全局坐标下的坐标转换到 车辆坐标系   phi为角度值
const XY& globalTocar_coordinate( XY global_coordinate,
                                    XY car_location_inglobal, 
                                    double phi) {
    // 角度转换为 弧度
    double radian = phi*M_PI/180;
    // 旋转矩阵 R
    Eigen::Matrix<double, 2,2> transform_R;

    transform_R << std::cos(radian), -std::sin(radian),
                    std::sin(radian), std::cos(radian);

    // std::cout<< "R    "<<transform_R<<'\n';
    // 逆矩阵
    Eigen::Matrix<double, 2, 2> R_inverse = transform_R.inverse();

    
    // 平移矩阵T
    // Eigen::Vector2d transfrom_T;
    //车辆坐标系在 全局坐标系中的坐标
    Eigen::Vector2d x1y1;
    // 点在全局坐标系中的坐标
    Eigen::Vector2d x2y2;
    // 转换结果
    Eigen::Vector2d result;

    x1y1 << car_location_inglobal.m_X,
            car_location_inglobal.m_Y;

    x2y2 << global_coordinate.m_X,
            global_coordinate.m_Y;

    result<< R_inverse*(x2y2-x1y1);
    //美化数据
    if (fabs(result[0,0]) <1e-10) {
        result[0,0] = 0;
    }
    else if (fabs(result[0,1]) < 1e-10) {
        result[0,1] = 0;
    }
    static XY xy_coor;
    xy_coor.m_X = result[0,0];
    xy_coor.m_Y = result[0,1];

    return xy_coor;
};

int indexLanePoint(Eigen::VectorXd &X, int &index_old, XY &carLocation) {
    
    int index;
    for (int i= index_old; i< X.size(); ++i) {

        if ( fabs(X[i]- carLocation.m_X) < 0.5) {

            index =i;
            // std::cout<< index <<'\n';
        }
    };
    // std::cout<< index <<'\n';
    return index;
};

// indexand 切片
void indexAndassign(int &index, const int &range, Eigen::VectorXd &X, Eigen::VectorXd &Y ,
                    Eigen::VectorXd &x, Eigen::VectorXd &y, XY &carlocation, double &carheadangle) {
    std::vector<double> x_vec;
    std::vector<double> y_vec;
    x_vec.reserve(65);
    y_vec.reserve(65);
    //fangzhi chaochu suoyin fanwei
    if ( index <130) {
        for (int i= index; i< index+range; ++i) {
            XY xy;
            XY xy_incarcoordinate;
            xy.m_X = X[i];
            xy.m_Y = Y[i];

            //转换到车辆坐标系下
            xy_incarcoordinate = globalTocar_coordinate(xy, carlocation, carheadangle);
            x_vec.push_back(xy_incarcoordinate.m_X);
            y_vec.push_back(xy_incarcoordinate.m_Y);
        }
    }
    else {
        for (int i= index; i< X.size(); ++i) {
            XY xy;
            XY xy_incarcoordinate;
            xy.m_X = X[i];
            xy.m_Y = Y[i];

            //转换到车辆坐标系下
            xy_incarcoordinate = globalTocar_coordinate(xy, carlocation, carheadangle);
            x_vec.push_back(xy_incarcoordinate.m_X);
            y_vec.push_back(xy_incarcoordinate.m_Y);
        }
    }
    
    x = Eigen::Map<Eigen::VectorXd>(x_vec.data(), x_vec.size());
    y = Eigen::Map<Eigen::VectorXd>(y_vec.data(), y_vec.size());
    // std::cout<< y[0]<<std::endl;
    // std::cout<< y[0]<<std::endl;

};

// 拟合三次多项式   y=a+bx+cx2+dx3
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order) 
{
    assert(xvals.size() == yvals.size());
    // assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
             A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// 系数美化赋值
const LaneFunc& assingcoeff( Eigen::VectorXd coeff) {
    static LaneFunc lanefuc;
    if ( fabs(coeff[3])< 1e-10) {
        lanefuc.A_LaneFunc = 0;
    }
    else {
        lanefuc.A_LaneFunc = coeff[3];
    }
    if ( fabs(coeff[2])< 1e-10) {
        lanefuc.B_LaneFunc = 0;
    }
    else {
        lanefuc.B_LaneFunc = coeff[2];
    }
    if ( fabs(coeff[1])< 1e-10) {
        lanefuc.C_LaneFunc = 0;
    }
    else {
        lanefuc.C_LaneFunc = coeff[1];
    }
    if ( fabs(coeff[0])< 1e-10) {
        lanefuc.D_LaneFunc = 0;
    }
    else {
        lanefuc.D_LaneFunc = coeff[0];
    }
    return lanefuc;
};


// 车辆坐标系下的坐标转换到全局坐标系
XY& cartoglobal_coordinate( const XY& xy_incar, const XY& car_Location, double phi) {
    
    static XY xy_global;
    
    // 角度转换为 弧度
    double radian = phi*M_PI/180;
    // 旋转矩阵R
    Eigen::Matrix<double, 2,2> transform_R;

    transform_R << std::cos(radian), -std::sin(radian),
                    std::sin(radian), std::cos(radian);
    // 平移矩阵T
    Eigen::Vector2d transfrom_T;
    // 车辆坐标系下点的坐标
    Eigen::Vector2d x1y1;
    // 转换结果
    Eigen::Vector2d result;

    transfrom_T << car_Location.m_X,
                    car_Location.m_Y;

    x1y1 << xy_incar.m_X,
            xy_incar.m_Y;

    result << transform_R*x1y1+transfrom_T;
    xy_global.m_X = result[0,0];
    xy_global.m_Y = result[0,1];
    return xy_global;
};

//加载环境中锥桶
void assignCone( Eigen::VectorXd &X, Eigen::VectorXd &Y, XY &car_location, std::vector<XY> &coneincarcoor, double &phi) {
    for (int i = 0; i<X.size(); ++i) {
        XY xy_cone;
        XY XY_cone_incarcoor;
        if (X[i] - car_location.m_X <30 && X[i]- car_location.m_X >0) {
            xy_cone.m_X = X[i];
            xy_cone.m_Y = Y[i];
            XY_cone_incarcoor = globalTocar_coordinate(xy_cone, car_location, phi);
            coneincarcoor.push_back(XY_cone_incarcoor);
        }
    }
};

// 基于三次样条曲线进行曲线平滑
const std::vector<XY>& SplineSmoother( const std::vector<XY>& rough_XY ) {
    std::vector<double> rough_X;
    std::vector<double> rough_Y;

    for (std::vector<XY>::const_iterator it = rough_XY.begin(); 
        it != rough_XY.end();
        ++it) {
        
        rough_X.push_back(it->m_X);
        rough_Y.push_back(it->m_Y);
    }

    tk::spline smoother(rough_X, rough_Y);

    std::vector<double> smoooth_X;
    std::vector<double> smoooth_Y;
    static std::vector<XY> smooth_XY;

    for (int i = 0; i<rough_X[rough_X.size() -1]; ++i) {
        smoooth_X.push_back(i);
        smoooth_Y.push_back(smoother(i));
    }

    Eigen::VectorXd smooth_x = Eigen::Map<Eigen::VectorXd>(smoooth_X.data(), smoooth_X.size());
    Eigen::VectorXd smooth_y = Eigen::Map<Eigen::VectorXd>(smoooth_Y.data(), smoooth_Y.size());
    for (int i=0; i<smooth_x.size(); ++i) {
        XY smooth_xy;
        smooth_xy.m_X = smooth_x[i];
        smooth_xy.m_Y = smooth_y[i];
        smooth_XY.push_back(smooth_xy);
    }

    return smooth_XY;
};

// 找到距离车的位置最小的点
void findNearestPoint( unsigned int& index_new, unsigned int& index_current ,double& dist2_old ,double& x, double& y) {

    double dist2_new = x*x + y*y;
    if ( dist2_new < dist2_old) {
        dist2_old = dist2_new;
        index_current = index_new;
    }
    else {
        index_current = index_current;
    }
};

// 对一阶函数求导
double Oneorderderivative(DPoint* child_point, DPoint* parent_point) {
    Eigen::Matrix<double, 2, 2> x;
    x << child_point->m_X, 1,
            parent_point->m_X, 1;
    
    Eigen::Matrix<double, 2, 1> y;
    y << child_point->m_Y,
            parent_point->m_Y;

    auto x_inverse = x.inverse();
    auto ab = x_inverse*y;
    // std::cout<< ab<<'\n';
    return ab(1,0);
};

// 根据pair中的损失进行排序 升序
bool sortBycost( std::pair<int, unsigned int >& pair1, std::pair<int, unsigned int >& pair2 ) {
    return pair1.first < pair2.first;
};

// 根据DPoint中的x值 排序 升序
bool sortByX(DPoint& point1, DPoint& point2) {
    return point1.m_X < point2.m_X;
}
// paixu tongguo juli
bool sortBydist(std::pair<double, unsigned int >& pair1, std::pair<double, unsigned int >& pair2 ) {
    return pair1.first < pair2.first;
}

// point3 next_point, point2 current_point , point1 current_point.m_parent
double sloveAngleofTwovec(DPoint* point1, DPoint* point2, DPoint* point3) {
    double theta = std::atan2(point1->m_X - point2->m_X, point1->m_Y - point2->m_Y) -
                    std::atan2(point3->m_X - point2->m_X, point3->m_Y - point2->m_Y);
    if ( theta > M_PI) {
        theta -= 2*M_PI;
    }
    else if ( theta < -M_PI) {
        theta += 2*M_PI;
    }
    // 弧度转换为角度
    theta = theta*180.0 / M_PI;
    return theta;
}

// // 计算车辆坐标系x轴与全局坐标系x轴的夹角
// double solveCurrentTheta(XY vehicle_head, XY vehicle_tail) {
//     double theta = std::atan2(-10, 0) - 
//                     std::atan2(vehicle_tail.m_X - vehicle_head.m_X, vehicle_tail.m_Y - vehicle_head.m_Y);
//                     // x轴
//     if ( theta > M_PI) {
//         theta -= 2*M_PI;
//     }
//     else if ( theta < -M_PI) {
//         theta += 2*M_PI;
//     }  
//     return theta;
// };

double Pi2Pi(double angle) {
    double a;
    if (angle > M_PI) {
        a = angle - 2 * M_PI;
    };

    if (angle < M_PI) {
        a = angle + 2 *M_PI;
    }
    return a;
};

double norm(XY val) {
    return std::sqrt(std::pow(val.m_X,2) + std::pow(val.m_Y,2));
};

double dot(XY val1, XY val2) {
    return val1.m_X*val1.m_Y + val2.m_X*val2.m_Y;
}

std::vector<double> SolveQuadraticEquation(double &a, double &b, double &c) {
    std::vector<double> root;
    root.reserve(2);

    double determine = std::pow(b,2) - 4*a*c;

    if (determine > 0) {
        double root1 = (-b + std::sqrt(std::pow(b,2)-4*a*c)) / (2*a);
        double root2 = (-b - std::sqrt(std::pow(b,2)-4*a*c)) / (2*a);

        root.push_back(root1);
        root.push_back(root2);
    }
    else if (determine == 0) {
        double root1 = -b / 2*a;
        root.push_back(root1);
    }
    else if (determine < 0) {
        std::cout<< "无实根"<< '\n';
    };
    return root;
}