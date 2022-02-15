#ifndef _UTILS_
#define _UTILS_


#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

#include "eigen3/Eigen/Dense"
#include "External/spline/src/spline.h"

#include "config.h"


double solveFunction(LaneFunc &lanefunc, float x);

bool CompareByD(LaneFunc &lanefunc1, LaneFunc &lanefunc2);

bool CompareByX(XY &xy1, XY &xy2);

float Calc_Dist(float x1, float x2, float y1, float y2);

double variance(std::vector<double> vec);

float findnearestconeX( std::vector<XY> xy);

const XY& globalTocar_coordinate( XY global_coordinate,
                                    XY car_location_inglobal, 
                                    double phi);

int indexLanePoint(Eigen::VectorXd &X, int &index_old, XY &carLocation);

void indexAndassign(int &index, const int &range, Eigen::VectorXd &X, Eigen::VectorXd &Y ,
                    Eigen::VectorXd &x, Eigen::VectorXd &y, XY &carlocation, double &carheadangle);

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order);

const LaneFunc& assingcoeff( Eigen::VectorXd coeff);

XY& cartoglobal_coordinate( const XY& xy_incar, const XY& car_Location, double phi);

void assignCone( Eigen::VectorXd &X, Eigen::VectorXd &Y, XY &car_location, std::vector<XY> &coneincarcoor, double &phi );

const std::vector<XY>& SplineSmoother( const std::vector<XY>& rough_XY );

void findNearestPoint( unsigned int& index_new, unsigned int& index_current, double& dist2_Old ,double& x, double& y );

double Oneorderderivative(DPoint* child_point, DPoint* parent_point);

bool sortBycost( std::pair<int, unsigned int >& pair1, std::pair<int, unsigned int >& pair2 );

bool sortByX(DPoint& point1, DPoint& point2 );

bool sortBydist(std::pair<double, unsigned int >& pair1, std::pair<double, unsigned int >& pair2 );

double sloveAngleofTwovec(DPoint* point1, DPoint* point2, DPoint* point3);

double solveCurrentTheta(XY vehicle_head, XY vehicle_tail);

double Pi2Pi(double angle);

double norm(XY val);

double dot(XY val1, XY val2);

std::vector<double> SolveQuadraticEquation(double &a, double &b, double &c);

#endif /*_UTILS_*/