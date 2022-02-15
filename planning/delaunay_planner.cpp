#include "delaunay_planner.h"

double Delaunay_Planner::Max_angle_change_cost = 599;
double Delaunay_Planner::Cone_gap_cost = 699; 
double Delaunay_Planner::Smooth_path_cost = 399; 
// double Delaunay_Planner::Sqrt_between_PathandSensor_cost = 0.2;
double Delaunay_Planner::Cone_gap = 2; // m


/**
 * 输入： 锥桶的XY坐标
 * 输出： 无
 * 作用： 构造函数 将XY 转换为 Point
**/
Delaunay_Planner::Delaunay_Planner(const std::vector<XY>& cone_vec) {
    this->Init( cone_vec );
};

/**
 * 输入： 锥桶XY坐标
 * 输出： 无
 * 作用： 初始化 将XY 转换为 Point
**/
void Delaunay_Planner::Init( std::vector<XY> cone_xy ) {

    unsigned int i=0; // 用以索引点

    // 将xy坐标，构建为Point点集
    for (std::vector<XY>::iterator it = cone_xy.begin();
            it != cone_xy.end();
                ++it) {
        this->points.push_back( std::make_pair( Point(it->m_X, it->m_Y), i) );
        ++i;
    }
};

/**
 * 输入： 无
 * 输出： 无
 * 作用： 使用德劳内三角剖分将空间离散化，并将每条边的中点 存进Delaunay_Middle_point中
**/
void Delaunay_Planner::Discrete_space_Find_middle_point() {

    // 德劳内三角剖分实例化
    Delaunay triangulation;
    triangulation.insert(points.begin(), points.end());  // 插入点集

    for (Delaunay::Finite_edges_iterator it = triangulation.finite_edges_begin();
            it != triangulation.finite_edges_end();
                ++it) {
        
        Delaunay::Edge e = *it;
        unsigned int index1 = e.first->vertex( (e.second+1)%3 )->info();
        unsigned int index2 = e.first->vertex( (e.second+2)%3 )->info();

        Edge edge;
        edge.first = index1;
        edge.last = index2;
        this->edge_vec.push_back(edge);

        DPoint middle_point;

        double x1 =0;
        double y1 =0;
        double x2=0;
        double y2 =0;
        for (std::vector< std::pair<Point, unsigned> >::iterator it = this->points.begin();
                it != this->points.end();
                    ++it) {
            
            if (it->second == index1) {
                x1 = it->first.x();
                y1 = it->first.y();
            }
            else if ( it->second == index2) {
                x2 = it->first.x();
                y2 = it->first.y();
            }
        }

        middle_point.m_X = (x1+x2)/2.0;
        middle_point.m_Y = (y1+y2)/2.0;
        middle_point.m_parent = nullptr;
        this->Delaunay_Middle_point.push_back(middle_point);
    }
};

/**
 * 输入： 无
 * 输出： 无
 * 作用： 根据每一个德劳内三角的边的中点 计算每个点的损失，规划路径，借鉴A*算法。
**/
std::vector<Path> Delaunay_Planner::Plan() {

    Discrete_space_Find_middle_point();

    std::vector<DPoint*> rough_traj; //粗轨迹点的指针
    rough_traj.reserve(30);

    DPoint start_point(1,6.875 ,nullptr);   // location change!!!!!!!!!!!!attention  1.6.875 ,4.1 8.6
    rough_traj.push_back(&start_point);

    // 根据 x 值的大小对中点进行排序 选出前三个点， 计算车辆到这三个点的距离，放进一个list中， 然后找到其中距离最小的点作为路径的第二个点
    std::sort(this->Delaunay_Middle_point.begin(), this->Delaunay_Middle_point.end(), sortByX);
    std::list< std::pair<double, unsigned int> > dist;

    for (int i=0; i<3; ++i) {
        dist.push_back( std::make_pair(Calc_Dist(this->Delaunay_Middle_point[i].m_X, start_point.m_X,
                                    this->Delaunay_Middle_point[i].m_Y, start_point.m_Y), i) );
    }
    auto min_dist_itear = std::min_element(dist.begin(), dist.end(), sortBydist);
    
    DPoint second_point = this->Delaunay_Middle_point[(*min_dist_itear).second];
    second_point.m_parent = &start_point;
    rough_traj.push_back(&second_point);

    // xiaoyu tance juli
    for ( DPoint x = second_point; x.m_X < 30.7;) {  // 28 wie dongtaizhi       30.7
        DPoint* next_point;

        next_point = calc_point_cost(&x);
        next_point->m_parent = rough_traj.back();
        rough_traj.push_back(next_point);

        x = *next_point;
    }

    // DPoint 转换为 XY
    std::vector<XY> rough_path;
    std::vector<Path> rough_path1;
    for (std::vector<DPoint*>::iterator it = rough_traj.begin();
            it != rough_traj.end();
                ++it) {
        XY xy;
        xy.m_X = (*it)->m_X;
        xy.m_Y = (*it)->m_Y;
        rough_path.push_back(xy);

        Path temp_path;
        temp_path.xy.m_X = (*it)->m_X;
        temp_path.xy.m_Y = (*it)->m_Y;
        rough_path1.push_back(temp_path);
    }
    
    // 平滑路径
    // BezierSmoother Smoother(rough_path);

    std::cout<< rough_path.size() << '\n';

    Bezier smoother(rough_path);


    std::vector<Path> smooth_path = smoother.Smooth();
    smoother.showpoints();
    return smooth_path;


    // return rough_path1;
};

/**
 * 输入： 无
 * 输出： 损失
 * 作用： 计算锥桶摆放间隔损失
**/
int Delaunay_Planner::calc_cost_G(DPoint* parent_point, DPoint* child_point) {

    double gap = (child_point->m_X - parent_point->m_X)*(child_point->m_X - parent_point->m_X) +
                    (child_point->m_Y - parent_point->m_Y)*(child_point->m_Y - parent_point->m_Y);

    int cost_G = (gap < 3.45 )
                    ?child_point->C_Gap - this->Cone_gap_cost
                    :child_point->C_Gap;
    return cost_G;
};

/**
 * 输入： 无
 * 输出： 损失
 * 作用： 计算角度损失 ， 锥桶车道的摆放 不能超过车辆的最大转向角
**/
int Delaunay_Planner::calc_cost_A(DPoint* parent_point, DPoint* child_point) {

    double theta = sloveAngleofTwovec(parent_point->m_parent, parent_point, child_point);
    // 180-40 =140
    int cost_a = ( abs(theta) > 140 )
                    ?child_point->C_Angle - this->Max_angle_change_cost
                    :child_point->C_Angle;
    return cost_a;
};

/**
 * 输入： 无
 * 输出： 损失
 * 作用： 计算 当前 4段路径的导数 的方差，方差小于某值  0.1  认为路径无突变 
**/
int Delaunay_Planner::calc_cost_D(DPoint* parent_point, DPoint* child_point) {

    std::vector<double> Derivative;
    Derivative.reserve(4);
    // 每条线段的导数
    double Derivative_line1 = Oneorderderivative( child_point, parent_point);

    Derivative.push_back(Derivative_line1);
    if ( parent_point->m_parent != nullptr) {
        double Derivative_line2 = Oneorderderivative(parent_point, parent_point->m_parent);
        Derivative.push_back(Derivative_line2);

        if ( parent_point->m_parent->m_parent != nullptr) {
            double Derivative_line3 = Oneorderderivative(parent_point->m_parent, parent_point->m_parent->m_parent);
            Derivative.push_back(Derivative_line3);

            if ( parent_point->m_parent->m_parent->m_parent != nullptr) {
                double Derivative_line4 = Oneorderderivative(parent_point->m_parent->m_parent,
                                                                parent_point->m_parent->m_parent->m_parent);
                Derivative.push_back(Derivative_line4);
            }
        }
    }  

    // // 导数的方差小于0.1
    int cost_d = (variance(Derivative) <0.1 )
                    ?child_point->C_Mid - this->Smooth_path_cost
                    :child_point->C_Mid;
    return cost_d;
}

/**
 * 输入： 无
 * 输出： 损失
 * 作用： 计算总损失返回 下一个点的指针
**/
DPoint* Delaunay_Planner::calc_point_cost(DPoint* current_Point) {
    // 存放 点的总损失， 以及对应点的索引
    std::vector< std::pair<int, unsigned int> > cost_Point;
    cost_Point.reserve(20);

    for (std::vector<DPoint>::iterator it = this->Delaunay_Middle_point.begin();
            it != this->Delaunay_Middle_point.end();
                ++it) {

        // 减少计算开销， 只搜索当前点之后2m内的点
        if ( it->m_X < current_Point->m_X+ 2 && it->m_X > current_Point->m_X ) {

            it->C_Gap = calc_cost_G(current_Point, &(*it));
            it->C_Angle = calc_cost_A(current_Point, &(*it));
            it->C_Mid = calc_cost_D(current_Point, &(*it));

            int all_cost = it->C_Mid+ it->C_Gap+ it->C_Angle;
            unsigned int index = std::distance(this->Delaunay_Middle_point.begin(), it);
            cost_Point.push_back(std::make_pair(all_cost, index));

            // reset default
            it->setDefaultCost();
        }
        else {
            continue;
        }
    }

    auto itear = std::min_element(cost_Point.begin(), cost_Point.end(), sortBycost);
    // 最近点的地址
    return &this->Delaunay_Middle_point[(*itear).second];
};

/**
 * 输入： 无
 * 输出： 中点数组
 * 作用： 返回中点数组
**/
std::vector<DPoint>& Delaunay_Planner::getMidPoints() {
    return this->Delaunay_Middle_point;
}

/**
 * 输入： 无
 * 输出： 点对数组
 * 作用： 返回点对数组
**/
std::vector< std::pair<Point, unsigned> >& Delaunay_Planner::getPoints() {
    return this->points;
}

/**
 * 输入： 无
 * 输出： 德劳内三角的边
 * 作用： 返回边数组
**/
std::vector<Edge>& Delaunay_Planner::getEdgevec() {
    return this->edge_vec;
}