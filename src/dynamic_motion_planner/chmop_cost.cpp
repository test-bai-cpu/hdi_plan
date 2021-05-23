#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "dynamic_motion_planner/chmop_cost.hpp"

namespace hdi_plan {

ChompCost::ChompCost(const ChmopTrajectory& trajectory, const std::vector<double>& derivative_costs, double ridge_factor) {
	int num_vars_all = trajectory.get_num_points();
	int num_vars_free = num_vars_all - 2 * (hdi_plan_utils::DIFF_RULE_LENGTH - 1);
	Eigen::MatrixXd diff_matrix = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);
	this->quad_cost_full_ = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);

	// construct the quad cost for all variables, as a sum of squared differentiation matrices
	double multiplier = 1.0;
	for (unsigned int i = 0; i < derivative_costs.size(); i++)
	{
		multiplier *= trajectory.get_discretization();
		diff_matrix = getDiffMatrix(num_vars_all, &hdi_plan_utils::DIFF_RULES[i][0]);
		this->quad_cost_full_ += (derivative_costs[i] * multiplier) * (diff_matrix.transpose() * diff_matrix);
	}
	this->quad_cost_full_ += Eigen::MatrixXd::Identity(num_vars_all, num_vars_all) * ridge_factor;

	// extract the quad cost just for the free variables:
	quad_cost_ = quad_cost_full_.block(DIFF_RULE_LENGTH - 1, DIFF_RULE_LENGTH - 1, num_vars_free, num_vars_free);

	// invert the matrix:
	quad_cost_inv_ = quad_cost_.inverse();
}


ChompCost::~ChompCost() = default;
}