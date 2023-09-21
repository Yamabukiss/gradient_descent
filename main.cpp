#include <iostream>
#include "solver.h"

int main() {
    auto obj = Solver(3);
    Eigen::Matrix4d dimension_mat;
    dimension_mat <<
                  -0.00283085,-0.999989,-0.00362801,66.4542,
            0.999869,-0.00288823,0.0159045,535.103,
            -0.0159148,-0.00358252,0.999867,416.882,
            0,0,0,1;

    Eigen::Vector4d cam_point1(210.3, 125.4, 642.1, 1);
    Eigen::Vector4d cam_point2(211.1, 125.9, 640.9, 1);

    Eigen::Vector4d base_point1(1008.65, -56.1, 306.52, 1);
    Eigen::Vector4d base_point2(1028.64, -68.19, 296.21, 1);

    std::vector<Eigen::Vector4d> base_points_vec = {base_point1, base_point2};
    std::vector<Eigen::Vector4d> cam_points_vec = {cam_point1, cam_point2};

    obj.estimationOptimize(dimension_mat, cam_points_vec, base_points_vec, 500);
    return 0;
}

