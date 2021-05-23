#pragma once

#include "dynamic_motion_planner/chmop_trajectory.hpp"
#include "utils/utility_functions.hpp"

namespace hdi_plan {
class ChompCost {
public:
	ChompCost(const ChmopTrajectory& trajectory, const std::vector<double>& derivative_costs, double ridge_factor = 0.0);
	~ChompCost();

private:
	Eigen::MatrixXd quad_cost_full_;
	Eigen::MatrixXd quad_cost_;
	Eigen::MatrixXd quad_cost_inv_;

	Eigen::MatrixXd getDiffMatrix(int size, const double* diff_rule) const;

};
}