#include "dynamic_motion_planner/chmop.hpp"

namespace hdi_plan {

Chmop::Chmop(const std::shared_ptr<ChmopTrajectory>& trajectory) {
	this->original_trajectory_ = trajectory;
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

	this->last_improvement_iteration_ = -1;


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

void Chmop::perform_forward_kinematics() {
	double inv_time = 1.0 / this->original_trajectory_->get_discretization();
	double inv_time_sq = inv_time * inv_time;

	// for each point in trajectory
	for (int i=1; i<=this->num_vars_; i++) {

	}

}

double Chmop::get_collision_cost() {
	double collision_cost = 0.0;
	return collision_cost;


}

double Chmop::get_smoothness_cost() {
	double smoothness_cost = 0.0;
	return smoothness_cost;
}

double Chmop::calculate_smoothness_increments() {
	return 0;
}

double Chmop::calculate_collision_increments() {
	return 0;
}

double Chmop::calculate_total_increments() {
	return 0;
}

void Chmop::add_increments_to_trajectory() {

}


}