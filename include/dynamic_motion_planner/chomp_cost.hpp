#pragma once

#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "dynamic_motion_planner/chomp_trajectory.hpp"
#include "utils/utility_functions.hpp"

namespace hdi_plan {
class ChompCost {
public:
	ChompCost(const std::shared_ptr<ChompTrajectory>& trajectory, const std::vector<double>& derivative_costs, double ridge_factor = 0.0);
	~ChompCost();

	double getMaxQuadCostInvValue() const {
		return this->quad_cost_inv_.maxCoeff();
	};
	Eigen::MatrixXd getQuadraticCostInverse() const {
		return this->quad_cost_inv_;
	}
	Eigen::MatrixXd getQuadraticCost() const {
		return this->quad_cost_;
	}
	void scale(double scale);
	double getCost(const Eigen::MatrixXd::ColXpr& joint_trajectory) const;

	Eigen::MatrixXd getDerivative(const Eigen::MatrixXd::ColXpr& joint_trajectory) const;
private:
	Eigen::MatrixXd quad_cost_full_;
	Eigen::MatrixXd quad_cost_;
	Eigen::MatrixXd quad_cost_inv_;

	Eigen::MatrixXd getDiffMatrix(int size, const double* diff_rule) const;

};
}