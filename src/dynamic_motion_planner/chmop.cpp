#include "dynamic_motion_planner/chmop.hpp"

namespace hdi_plan {

Chmop::Chmop(const std::shared_ptr<ChmopTrajectory>& trajectory, const std::map<std::string, std::shared_ptr<Obstacle>>& obstacle_map) {
	this->original_trajectory_ = trajectory;
	this->obstacle_map_ = obstacle_map;
	this->initialize();
	this->optimize();
}

Chmop::~Chmop() = default;

void Chmop::initialize() {
	this->num_vars_ = this->original_trajectory_->get_num_points();

	// allocate memory for matrices:
	this->smoothness_increments_ = Eigen::VectorXd::Zero(this->num_vars_);
	this->collision_increments_ = Eigen::VectorXd::Zero(this->num_vars_);
	this->final_increments_ = Eigen::VectorXd::Zero(this->num_vars_);
	this->smoothness_derivative_ = Eigen::VectorXd::Zero(this->num_vars_);

	this->collision_point_pos_.resize(this->num_vars_);
	this->collision_point_vel_.resize(this->num_vars_);
	this->collision_point_acc_.resize(this->num_vars_);
	this->collision_point_vel_mag_.resize(this->num_vars_);
	this->collision_point_potential_.resize(this->num_vars_);
	this->collision_point_potential_gradient_.resize(this->num_vars_);
	this->last_improvement_iteration_ = -1;

	this->get_collision_point_pos();

}

bool Chmop::optimize() {
	bool optimization_result = false;
	for (int iteration=0; iteration<this->max_iterations_; iteration++) {
		perform_forward_kinematics();
		double c_cost = this->get_collision_cost();
		double s_cost = this->get_smoothness_cost();
		double cost = c_cost + s_cost;

		if (iteration == 0 || cost < this->best_trajectory_cost_) {
			this->best_trajectory_cost_ = cost;
			this->last_improvement_iteration_ = iteration;
		}

		this->calculate_smoothness_increments();
		this->calculate_collision_increments();
		this->calculate_total_increments();
		this->add_increments_to_trajectory();
	}
	return optimization_result;
}

void Chmop::get_collision_point_pos() {
	for (int i=1; i<=this->num_vars_; i++) {
		this->collision_point_pos_[i-1] = this->original_trajectory_->get_position_by_index(i);
	}
}

void Chmop::perform_forward_kinematics() {
	double inv_time = 1.0 / this->original_trajectory_->get_discretization();
	double inv_time_sq = inv_time * inv_time;

	this->is_collsion_free_ = true;

	for (int i=0; i<this->num_vars_; i++) {
		this->collision_point_potential_[i] = this->get_potential(this->collision_point_pos_[i]);
		this->collision_point_potential_gradient_[i] = Eigen::Vector3d(0,0,0);

		this->collision_point_vel_[i] = Eigen::Vector3d(0, 0, 0);
		this->collision_point_acc_[i] = Eigen::Vector3d(0, 0, 0);
		for (int k = -hdi_plan_utils::DIFF_RULE_LENGTH / 2; k <= hdi_plan_utils::DIFF_RULE_LENGTH / 2; k++)
		{
			collision_point_vel_[i] +=
					(inv_time * hdi_plan_utils::DIFF_RULES[0][k + hdi_plan_utils::DIFF_RULE_LENGTH / 2]) * this->collision_point_pos_[i + k];
			collision_point_acc_[i] +=
					(inv_time_sq * hdi_plan_utils::DIFF_RULES[1][k + hdi_plan_utils::DIFF_RULE_LENGTH / 2]) * this->collision_point_pos_[i + k];
		}
		// get the norm of the velocity:
		this->collision_point_vel_mag_[i] = this->collision_point_vel_[i].norm();
	}
}

double Chmop::get_potential(const Eigen::Vector3d& point) {
	double distance_to_nearest_obstacle = std::numeric_limits<double>::infinity();
	for (auto obstacle : this->obstacle_map_) {
		double distance = hdi_plan_utils::get_distance(point, obstacle.second->get_position()) - this->drone_radius_ - (obstacle.second->get_size()/2);
		if (distance < distance_to_nearest_obstacle) distance_to_nearest_obstacle = distance;
	}

	if (distance_to_nearest_obstacle >= this->min_clearence_) {
		return 0.0;
	} else if (distance_to_nearest_obstacle >= 0.0) {
		const double diff = distance_to_nearest_obstacle - this->min_clearence_;
		const double gradient_magnitude = diff / this->min_clearence_;
		return 0.5 * gradient_magnitude * diff;
	} else {
		this->is_collsion_free_ = false;
		return -distance_to_nearest_obstacle + 0.5 * this->min_clearence_;
	}
}

double Chmop::get_collision_cost() {
	double collision_cost = 0.0;
	double worst_collision_cost = 0.0;
	this->worst_collision_cost_state_ = -1;

	for (int i=0; i<this->num_vars_; i++) {
		double state_collision_cost = this->collision_point_potential_[i] * this->collision_point_vel_mag_[i];
		collision_cost += state_collision_cost;
		if (state_collision_cost > worst_collision_cost)
		{
			worst_collision_cost = state_collision_cost;
			this->worst_collision_cost_state_ = i;
		}
	}
	return this->obstacle_cost_weight_ * collision_cost;


}

double Chmop::get_smoothness_cost() {
	double smoothness_cost = 0.0;
	return this->smoothness_cost_weight_ * smoothness_cost;
}

double Chmop::calculate_smoothness_increments() {
	return 0;
}

double Chmop::calculate_collision_increments() {
	double potential;
	double vel_mag_sq;
	double vel_mag;
	Eigen::Vector3d potential_gradient;
	Eigen::Vector3d normalized_velocity;
	Eigen::Matrix3d orthogonal_projector;
	Eigen::Vector3d curvature_vector;
	Eigen::Vector3d cartesian_gradient;

	this->collision_increments_.setZero(this->num_vars_);


	return 0;
}

double Chmop::calculate_total_increments() {
	return 0;
}

void Chmop::add_increments_to_trajectory() {

}


}