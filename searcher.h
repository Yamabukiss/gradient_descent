#pragma once
#include <iostream>
#include <Eigen/Dense>

class Searcher {
public:
    explicit Searcher(double step, int max_timeout_count = 2);
    ~Searcher();

    void backTrackLineSearch(
            const double &gradient, const double &error, int index,
            const Eigen::Vector4d& dimension_vec, const Eigen::MatrixXd &camera_mat,
            int try_times = 10, double alpha = 1e-2, double beta = 0.1);

    double step_;

private:
    static double calculateErrorPerDimension(const Eigen::Vector4d &dimension_vec,
                                      const Eigen::MatrixXd &camera_mat);

    int timeout_count_;
    int max_timeout_count_;
};


