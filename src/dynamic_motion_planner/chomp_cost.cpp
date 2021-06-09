#include "dynamic_motion_planner/chomp_cost.hpp"

namespace hdi_plan {

ChompCost::ChompCost(const std::shared_ptr<ChompTrajectory>& trajectory, const std::vector<double> &derivative_costs,
					 double ridge_factor) {
	ROS_INFO("ChompCost: start");
	int num_vars_all = trajectory->get_num_points_diff();
	int num_vars_free = num_vars_all - 2 * (hdi_plan_utils::DIFF_RULE_LENGTH - 1);
	Eigen::MatrixXd diff_matrix = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);
	this->quad_cost_full_ = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);

	// construct the quad cost for all variables, as a sum of squared differentiation matrices
	double multiplier = 1.0;
	for (unsigned int i = 0; i < derivative_costs.size(); i++) {
		multiplier *= trajectory->get_discretization();
		diff_matrix = getDiffMatrix(num_vars_all, &hdi_plan_utils::DIFF_RULES[i][0]);
		this->quad_cost_full_ += (derivative_costs[i] * multiplier) * (diff_matrix.transpose() * diff_matrix);
	}
	this->quad_cost_full_ += Eigen::MatrixXd::Identity(num_vars_all, num_vars_all) * ridge_factor;

	// extract the quad cost just for the free variables:
	quad_cost_ = quad_cost_full_.block(hdi_plan_utils::DIFF_RULE_LENGTH - 1, hdi_plan_utils::DIFF_RULE_LENGTH - 1, num_vars_free, num_vars_free);

	// invert the matrix:
	quad_cost_inv_ = quad_cost_.inverse();
	ROS_INFO("ChompCost: finish");
}

ChompCost::~ChompCost() = default;

Eigen::MatrixXd ChompCost::getDiffMatrix(int size, const double* diff_rule) const
{
	Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size, size);
	for (int i = 0; i < size; i++)
	{
		for (int j = -hdi_plan_utils::DIFF_RULE_LENGTH / 2; j <= hdi_plan_utils::DIFF_RULE_LENGTH / 2; j++)
		{
			int index = i + j;
			if (index < 0)
				continue;
			if (index >= size)
				continue;
			matrix(i, index) = diff_rule[j + hdi_plan_utils::DIFF_RULE_LENGTH / 2];
		}
	}
	return matrix;
}

void ChompCost::scale(double scale)
{
	double inv_scale = 1.0 / scale;
	this->quad_cost_inv_ *= inv_scale;
	this->quad_cost_ *= scale;
	this->quad_cost_full_ *= scale;
}

double ChompCost::getCost(const Eigen::MatrixXd::ColXpr &joint_trajectory) const {
	return joint_trajectory.dot(this->quad_cost_full_ * joint_trajectory);
}

Eigen::MatrixXd ChompCost::getDerivative(const Eigen::MatrixXd::ColXpr &joint_trajectory) const {
	Eigen::MatrixXd derivative = this->quad_cost_full_ * (2.0 * joint_trajectory);
	return derivative;
}

}