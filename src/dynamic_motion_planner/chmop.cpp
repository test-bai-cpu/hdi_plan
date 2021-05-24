#include "dynamic_motion_planner/chmop.hpp"

namespace hdi_plan {

Chmop::Chmop(const std::shared_ptr<ChmopTrajectory>& trajectory, const std::map<std::string, std::shared_ptr<Obstacle>>& obstacle_map) {
	this->full_trajectory_ = trajectory;
	this->obstacle_map_ = obstacle_map;
	this->initialize();
	this->optimize();
}

Chmop::~Chmop() = default;

void Chmop::initialize() {
	this->num_vars_all_ = this->full_trajectory_->get_num_points_diff(); // actual points + 10
	this->num_vars_free_ = this->full_trajectory_->get_num_points_free(); // actual points - 2
	this->num_vars_origin_ = this->full_trajectory_->get_num_points(); // actual points
	this->free_vars_start_ = this->full_trajectory_->get_start_index();
	this->free_vars_end_ = this->full_trajectory_->get_end_index();

	// get joint cost
	std::vector<double> derivative_costs(3);
	derivative_costs[0] = this->smoothness_cost_velocity_;
	derivative_costs[1] = this->smoothness_cost_acceleration_;
	derivative_costs[2] = this->smoothness_cost_jerk_;
	this->joint_cost_ = std::make_shared<ChompCost>(this->full_trajectory_, derivative_costs, this->ridge_factor_);
	double cost_scale = this->joint_cost_->getMaxQuadCostInvValue();
	this->joint_cost_->scale(cost_scale);

	// allocate memory for matrices:
	this->smoothness_increments_ = Eigen::MatrixXd::Zero(this->num_vars_free_, 1);
	this->collision_increments_ = Eigen::MatrixXd::Zero(this->num_vars_free_, 1);
	this->final_increments_ = Eigen::MatrixXd::Zero(this->num_vars_free_, 1);
	this->smoothness_derivative_ = Eigen::VectorXd::Zero(this->num_vars_all_);

	this->collision_point_pos_.resize(this->num_vars_all_);
	this->collision_point_vel_.resize(this->num_vars_all_);
	this->collision_point_acc_.resize(this->num_vars_all_);
	this->collision_point_vel_mag_.resize(this->num_vars_all_);
	this->collision_point_potential_.resize(this->num_vars_all_);
	this->collision_point_potential_gradient_.resize(this->num_vars_all_);

	this->jacobian_ = Eigen::MatrixXd::Zero(3, 1);
	this->jacobian_pseudo_inverse_ = Eigen::MatrixXd::Zero(1, 3);
	this->jacobian_jacobian_tranpose_ = Eigen::MatrixXd::Zero(3, 3);


	this->last_improvement_iteration_ = -1;
	this->get_collision_point_pos();

}

bool Chmop::optimize() {
	bool optimization_result = false;
	for (this->iteration_=0; this->iteration_ < this->max_iterations_; this->iteration_++) {
		perform_forward_kinematics();
		double c_cost = this->get_collision_cost();
		double s_cost = this->get_smoothness_cost();
		double cost = c_cost + s_cost;

		if (this->iteration_ == 0 || cost < this->best_trajectory_cost_) {
			this->best_trajectory_cost_ = cost;
			this->last_improvement_iteration_ = this->iteration_;
		}

		this->calculate_smoothness_increments();
		this->calculate_collision_increments();
		this->calculate_total_increments();
		this->add_increments_to_trajectory();
	}
	return optimization_result;
}

void Chmop::get_collision_point_pos() {
	int start_extra = this->full_trajectory_->get_start_extra();
	int end_extra = this->full_trajectory_->get_end_extra();
	for (int i=1; i<=this->num_vars_origin_; i++) {
		this->collision_point_pos_[i-1+start_extra] = this->full_trajectory_->get_position_by_index(i);
	}
	for (int i=0; i<start_extra; i++) {
		this->collision_point_pos_[i] = this->collision_point_pos_[start_extra];
	}
	for (int i=this->num_vars_all_-end_extra; i<this->num_vars_all_; i++) {
		this->collision_point_pos_[i] = this->collision_point_pos_[this->num_vars_all_-end_extra-1];
	}
}

void Chmop::perform_forward_kinematics() {
	double inv_time = 1.0 / this->full_trajectory_->get_discretization();
	double inv_time_sq = inv_time * inv_time;

	this->is_collsion_free_ = true;

	int start = this->free_vars_start_;
	int end = this->free_vars_end_;
	if (this->iteration_ == 0) {
		start = 0;
		end = this->num_vars_all_ - 1;
	}

	for (int i = start; i <= end; i++) {
		this->collision_point_potential_[i] = this->get_potential(this->collision_point_pos_[i]);
		// Todo: How to calculate the potential gradient
		this->collision_point_potential_gradient_[i] = Eigen::Vector3d(0,0,0);
	}

	for (int i = this->free_vars_start_; i <= this->free_vars_end_; i++) {
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

	for (int i=this->free_vars_start_; i<this->free_vars_end_; i++) {
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
	double smoothness_cost = this->joint_cost_.getCost();
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

	this->collision_increments_.setZero(this->num_vars_free_, 1);

	int start_point = this->free_vars_start_;
	int end_point = this->free_vars_end_;
	if (this->use_stochastic_descent_) {
		start_point = static_cast<int>(hdi_plan_utils::get_random_double() * (this->free_vars_end_ - this->free_vars_start_) + this->free_vars_start_);
		end_point = start_point;
	}

	for (int i = start_point; i <= end_point; i++) {
		potential = collision_point_potential_[i];
		if (potential < 0.0001)
			continue;
		potential_gradient = -collision_point_potential_gradient_[i];
		vel_mag = collision_point_vel_mag_[i];
		vel_mag_sq = vel_mag * vel_mag;

		normalized_velocity = this->collision_point_vel_[i] / vel_mag;
		orthogonal_projector = Eigen::Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
		curvature_vector = (orthogonal_projector * this->collision_point_acc_[i]) / vel_mag_sq;
		cartesian_gradient = vel_mag * (orthogonal_projector * potential_gradient - potential * curvature_vector);

		this->collision_increments_.row(i - this->free_vars_start_) -= Eigen::MatrixXd::Ones(1,3) * cartesian_gradient;
	}

	return 0;
}

/*
void Chmop::get_jacobian(int trajectory_point, const Eigen::Vector3d &collision_point_pos) {
	if (isParent(joint_name, joint_names_[j]))
	{
		Eigen::Vector3d column = joint_axes_[trajectory_point].cross(collision_point_pos - joint_positions_[trajectory_point]);

		this->jacobian_.col(0)[0] = column.x();
		this->jacobian_.col(0)[1] = column.y();
		this->jacobian_.col(0)[2] = column.z();
	}
	else
	{
		this->jacobian_.col(0)[0] = 0.0;
		this->jacobian_.col(0)[1] = 0.0;
		this->jacobian_.col(0)[2] = 0.0;
	}
}*/


double Chmop::calculate_total_increments() {
	return 0;
}

void Chmop::add_increments_to_trajectory() {

}



}