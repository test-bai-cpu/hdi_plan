#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <vector>
#include <memory>

#include "dynamic_motion_planner/chmop_trajectory.hpp"

namespace hdi_plan {
class Chmop {
public:
	Chmop(const std::shared_ptr<ChmopTrajectory>& trajectory);
	~Chmop();

	std::vector<Eigen::Vector3d> get_optimized_trajectory() const {
		return this->optimized_trajectory_;
	}

private:
	std::shared_ptr<ChmopTrajectory> original_trajectory_;
	std::vector<Eigen::Vector3d> optimized_trajectory_;
	double best_trajectory_cost_;
	int num_vars_;
	int last_improvement_iteration_;

	// matrix
	Eigen::VectorXd smoothness_increments_;
	Eigen::VectorXd collision_increments_;
	Eigen::VectorXd final_increments_;

	// temporary variables
	Eigen::VectorXd smoothness_derivative_;
	Eigen::Vector3d jacobian_;
	Eigen::Vector3d jacobian_pseudo_inverse_;
	Eigen::Matrix3d jacobian_jacobian_tranpose_;

	// collision cost


	// optimize process
	void initialize();
	bool optimize();

	void perform_forward_kinematics();
	double get_collision_cost();
	double get_smoothness_cost();
	double calculate_smoothness_increments();
	double calculate_collision_increments();
	double calculate_total_increments();
	void add_increments_to_trajectory();

	// parameters
	double planning_time_limit_{6.0};
	int max_iterations_{50};
	int max_iterations_after_collision_free_{5};
	double smoothness_cost_weight_{0.1};
	double obstacle_cost_weight_{1.0};
	double learning_rate_{0.01};
	double smoothness_cost_velocity_{0.0};
	double smoothness_cost_acceleration_{1.0};
	double smoothness_cost_jerk_{0.0};
	double righe_factor_{0.0};
};

}