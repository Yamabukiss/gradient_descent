#include "solver.h"

Solver::Solver(double precision, double init_trans_step) : init_trans_step_(init_trans_step)
{
    precision_ = pow(10, precision);
}

bool Solver::estimationOptimize(
        Eigen::Matrix4d dimension_mat,
        const std::vector<Eigen::Vector4d>& cam_points_vec,
        const std::vector<Eigen::Vector4d>& base_points_vec,
        int iterations) {

    size_t num_points = cam_points_vec.size();
    Eigen::MatrixXd cam_points_mat(4, static_cast<int>(num_points));
    Eigen::MatrixXd base_points_mat(3, static_cast<int>(num_points));

    for (size_t i = 0; i < num_points; i++)
        cam_points_mat.col(static_cast<int>(i)) = cam_points_vec[i];

    for (size_t i = 0; i < num_points; i++)
        base_points_mat.col(static_cast<int>(i)) = base_points_vec[i].head(3);

    for (int dimension = 0; dimension < dimension_mat.rows() - 1; dimension++)
    {
        double cost = 0, last_cost = 0;

        Searcher w_searcher(init_trans_step_);

        for (int iteration = 0; iteration < iterations; iteration++)
        {
            auto tmp_mat = dimension_mat * cam_points_mat;
            auto estimate_cam_mat = tmp_mat.block(0, 0, 3, num_points);
            auto error_mat = (base_points_mat - estimate_cam_mat).array().square() / 2;
            auto dimension_error = (base_points_mat - estimate_cam_mat).row(dimension);
            cost = error_mat.row(dimension).sum();

            double &w = dimension_mat(dimension, 3);

            double gradient_w = 0;

            for (int i = 0; i < num_points; i++)
                gradient_w += -dimension_error[i];

            w_searcher.backTrackLineSearch(gradient_w, cost, 3,
                                           dimension_mat.row(dimension), cam_points_mat);

            w -= gradient_w * w_searcher.step_;

            std::cout << "cost: " << cost << std::endl;
            std::cout << "last_cost: " << last_cost << std::endl;

            if ((iteration > 0 && (abs(std::round(cost * precision_)) >= abs(std::round(last_cost * precision_)))) || abs(gradient_w * w_searcher.step_) < 0.1)
            {
                std::cout << "----------------" << std::endl;
                std::cout << "error: " << cost << std::endl;
                std::cout << "dimension: " << dimension << std::endl;
                std::cout << "point1: " << (dimension_mat * cam_points_vec[0]) << std::endl;
                std::cout << "point2: " << (dimension_mat * cam_points_vec[1]) << std::endl;

                if (std::isnan(cost) || std::isinf(cost))
                {
                    std::cout << dimension << std::endl;
                    return false;
                }
                else
                    break;
            }

            last_cost = cost;
        }
        std::cout << "------iteration_end----------" << std::endl;
        std::cout << "error: " << cost << std::endl;
        std::cout << "dimension: " << dimension_mat.row(0) << std::endl;
        std::cout << "point1: " << (dimension_mat * cam_points_vec[0]) << std::endl;
        std::cout << "point2: " << (dimension_mat * cam_points_vec[1]) << std::endl;
    }
    return true;
}

Solver::~Solver()  = default;
