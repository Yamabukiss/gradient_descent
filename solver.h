#pragma once
#include "searcher.h"

class Solver
{
public:
    explicit Solver(double precision, double init_trans_step = 0.5);
    ~Solver();
    bool estimationOptimize(
    Eigen::Matrix4d dimension_mat,
    const std::vector<Eigen::Vector4d>& cam_points_vec,
    const std::vector<Eigen::Vector4d>& base_points_vec,
    int iterations = 500);

private:
    double precision_;
    double init_trans_step_;
};


