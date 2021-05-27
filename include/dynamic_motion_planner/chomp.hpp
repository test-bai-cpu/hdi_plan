#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <map>
#include <limits>
#include <vector>
#include <memory>

#include "generate_dynamic_scene/obstacle.hpp"
#include "dynamic_motion_planner/chomp_trajectory.hpp"
#include "dynamic_motion_planner/chomp_cost.hpp"
#include "generate_dynamic_scene/obstacle.hpp"
#include "utils/utility_functions.hpp"

namespace hdi_plan {
class Chomp {
public:
	Chomp(const std::shared_ptr<ChompTrajectory>& trajectory, const std::map<std::string, std::shared_ptr<Obstacle>>& obstacle_map);
	~Chomp();

	std::vector<Eigen::Vector3d> get_optimized_trajectory() const {
		return this->optimized_trajectory_;
	}

private:
	std::shared_ptr<ChompTrajectory> full_trajectory_;
	std::vector<Eigen::Vector3d> optimized_trajectory_;
	double best_trajectory_cost_;
	int num_vars_all_;
	int num_vars_free_;
	int num_vars_origin_;
	int free_vars_start_;
	int free_vars_end_;
	int last_improvement_iteration_;
	bool is_collsion_free_{true};
	int iteration_{0};

	// joint
	int num_joints_{3};
	std::vector<std::shared_ptr<ChompCost>> joint_costs_;

	// matrix
	Eigen::MatrixXd smoothness_increments_;
	Eigen::MatrixXd collision_increments_;
	Eigen::MatrixXd final_increments_;

	// temporary variables
	Eigen::VectorXd smoothness_derivative_;
	Eigen::MatrixXd jacobian_;
	Eigen::MatrixXd jacobian_pseudo_inverse_;
	Eigen::Matrix3d jacobian_jacobian_tranpose_;

	// smoothness cost
	//std::shared_ptr<ChompCost> joint_cost_;

	// collision cost
	std::map<std::string, std::shared_ptr<Obstacle>> obstacle_map_;
	std::vector<Eigen::Vector3d> collision_point_pos_;
	std::vector<Eigen::Vector3d> collision_point_vel_;
	std::vector<Eigen::Vector3d> collision_point_acc_;
	std::vector<double> collision_point_potential_;
	std::vector<double> collision_point_vel_mag_;
	std::vector<Eigen::Vector3d> collision_point_potential_gradient_;

	double get_potential(const Eigen::Vector3d& point);
	void get_collision_point_pos();
	//void get_jacobian(int trajectory_point, const Eigen::Vector3d& collision_point_pos);


	// optimize process
	void initialize();
	bool optimize();

	void perform_forward_kinematics();
	double get_collision_cost();
	double get_smoothness_cost();
	void calculate_smoothness_increments();
	void calculate_collision_increments();
	void calculate_total_increments();
	void add_increments_to_trajectory();

	int worst_collision_cost_state_{-1};

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
	double ridge_factor_{0.0};
	double drone_radius_{0.5};
	double min_clearence_{0.2};
	double joint_update_limit_{0.1};
	bool use_stochastic_descent_{false};


};

}