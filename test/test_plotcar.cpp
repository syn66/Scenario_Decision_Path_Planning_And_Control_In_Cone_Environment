#include <vector>
#include <cmath>
#include "External/matplotlib/matplotlibcpp.h"

namespace plt = matplotlibcpp;


int main()
{
    const int n = 20;
    std::vector<std::vector<double>> matrix;

    // for (int i = 0; i < n; ++i) {
    //     std::vector<double> row;
    //     for (int j = 0; j < n; ++j) {
    //         if (i == j)
    //             row.push_back(-2);
    //         else if (j == i - 1 || j == i + 1)
    //             row.push_back(1);
    //         else
    //             row.push_back(0);
    //     }
    //     matrix.push_back(row);
    // }

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> c;

    c.push_back(1);

    x.push_back(1);
    x.push_back(2);
    x.push_back(3);
    x.push_back(4);
    x.push_back(5);
    x.push_back(6);

    y.push_back(1);
    y.push_back(2);
    y.push_back(3);
    y.push_back(4);
    y.push_back(5);
    y.push_back(6);
        std::vector<double> row;

    matrix.push_back(x);
    matrix.push_back(y);

    // plt::spy(matrix, 5, {{"marker", "o"}});

    plt::scatter(x,y, 2);
    plt::show();

    return 0;
}