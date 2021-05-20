#include "dynamic_motion_planner/chmop_trajectory.hpp"

namespace hdi_plan {
ChmopTrajectory::ChmopTrajectory(const std::vector<Eigen::Vector3d>& trajectory_points) {
	this->trajectory_points_ = trajectory_points;
	this->calculate_duration_and_points_num_for_whole_trajectory();
}
ChmopTrajectory::ChmopTrajectory(Eigen::Vector3d start_point, Eigen::Vector3d end_point) {
	this->duration_ = this->get_distance_between_two_points(start_point, end_point) / this->speed_;
	this->num_points_ = static_cast<int>(this->duration_ / this->discretization_);
}

ChmopTrajectory::~ChmopTrajectory() = default;

double ChmopTrajectory::get_distance_between_two_points(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2) {
	return static_cast<double>(std::sqrt((point1 - point2).squaredNorm()));
}

void ChmopTrajectory::calculate_duration_and_points_num_for_whole_trajectory() {
	double discretize_length = this->discretization_ * this->speed_;
	int count = 0;
	double duration = 0.0;
	int total_number_of_points = 0;
	Eigen::Vector3d last_point;

	for (auto path_point : this->trajectory_points_) {
		if (count == 0) {
			last_point = path_point;
			count += 1;
			continue;
		}
		double distance = this->get_distance_between_two_points(last_point, path_point);
		duration += distance / this->speed_;
		if (distance < discretize_length) {
			total_number_of_points += 1;
			this->group_number_.push_back(1);
			count += 1;
			continue;
		}
		int number_of_interval_points = static_cast<int>(floor(distance/discretize_length));
		total_number_of_points += number_of_interval_points;
		this->group_number_.push_back(number_of_interval_points); // the number of group_number is vector(trajectory_points_) size -1
		count += 1;
	}

	this->num_points_ = total_number_of_points + 1;
	this->duration_ = duration;
	int total_num_temp = 0;
	for (int num : this->group_number_) {
		total_num_temp += num;
		this->group_number_add_.push_back(total_num_temp);
	}
}

Eigen::Vector3d ChmopTrajectory::get_position_by_index(int index) {
	// the index is start from 1
	for (int num : this->group_number_add_) {
		if (index > num) {

		}
	}
}

}

