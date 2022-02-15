#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>
#include <cmath>
#include "eigen3/Eigen/Dense"

#include 

struct XY
{
    double m_X;
    double m_Y;


};

void ss(int &a) {
    a = 10;
}

int test01() {
    int a;
    for (int i=0; i<6;++i) {
        if (i == 3) {
            a = i;
        }
    }
    return a;
};

const XY& globalTocar_coordinate( XY global_coordinate,
                                    XY car_location_inglobal, 
                                    double phi) {
    // XY car_coor;
    // 角度转换为 弧度
    double radian = phi*M_PI/180;
    // 旋转矩阵R
    Eigen::Matrix<double, 2,2> transform_R;

    transform_R << std::cos(radian), -std::sin(radian),
                    std::sin(radian), std::cos(radian);
    // 逆矩阵
    Eigen::Matrix<double, 2, 2> R_inverse = transform_R.inverse();

    
    // 平移矩阵T
    Eigen::Vector2d transfrom_T;
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

int main()
{

    XY test1;
    test1.m_X = 4.0;
    test1.m_Y = 5.0;

        XY test3;
    test3.m_X = 5.0;
    test3.m_Y = 5.0;

    XY test4;
    test4.m_X = 5.0;
    test4.m_Y = 4.0;

    XY test2;
    test2.m_X=2.0;
    test2.m_Y=2.0;


    XY car_coo1r;

    std::vector<XY> test;
    test.push_back(test1);
    test.push_back(test3);
    test.push_back(test4);

    car_coo1r = globalTocar_coordinate(test1, test2, 45);

    std::cout<< car_coo1r.m_X << "   "<< car_coo1r.m_Y <<std::endl;
        // std::cout<< car_coor[1].m_X << "   "<< car_coor[1].m_Y <<std::endl;
        //     std::cout<< car_coor[2].m_X << "   "<< car_coor[2].m_Y <<std::endl;

// Eigen::Matrix<double, 2,2> transform_R1;
// double radian = 45*M_PI/180;
//     transform_R1 << std::cos(radian), -std::sin(radian),
//                     std::sin(radian), std::cos(radian);

//     std::cout<< transform_R1<<std::endl;

//     Eigen::Matrix<double, 2, 2> R_inverse1 = transform_R1.inverse();

//         std::cout<< R_inverse1<<std::endl;


//         Eigen::Vector2d aa;
//         aa << 1,
//                 2;

//         std::cout<< R_inverse1*aa<<std::endl;

//                 Eigen::Vector2d result;
//         result<< R_inverse1*aa;
// std::cout<<result<<std::endl;
        
                
    // int a =45;
    // double radian = a*M_PI /180;
    // std::cout<< std::sin(radian) <<std::endl;
    // // double c =1;
    // // double d = 3;
    // double b = static_cast<double>(1)/ static_cast<double>(3);
    //     std::cout<< b <<std::endl;

    // Eigen::VectorXd aaa;
    // std::vector<int> bbb;
    // bbb.reserve(12);
    // for (int i =0 ; i<10;++i) {
    //     bbb.push_back(i);
    // }

    // aaa = Eigen::Map<Eigen::VectorXd>(bbb.data(), bbb.size());

    // std::cout <<aaa<<std::endl;

    // int c = 0;
    // ss(c);
    // std::cout<< c<<std::endl;
    int tes1 = test01();
    std::cout <<tes1<<std::endl;
}