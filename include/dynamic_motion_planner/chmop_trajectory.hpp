#pragma once

#include <math.h>
#include <vector>
#include <Eigen/Dense>

namespace hdi_plan {
class ChmopTrajectory {
public:
	ChmopTrajectory(const std::vector<Eigen::Vector3d>& trajectory_points);
	ChmopTrajectory(Eigen::Vector3d start_point, Eigen::Vector3d end_point);
	~ChmopTrajectory();

	int get_num_points() const {
		return this->num_points_;
	};

	int get_duration() const {
		return this->duration_;
	};

	double get_discretization() const {
		return this->discretization_;
	}

	Eigen::Vector3d get_position_by_index(int index);

private:
	std::vector<Eigen::Vector3d> trajectory_points_;
	double duration_;
	int num_points_;
	double discretization_{0.01}; // every interval is 0.01s
	double speed_{1.0};
	std::vector<int> group_number_;
	std::vector<int> group_number_add_;

	double get_distance_between_two_points(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);
	void calculate_duration_and_points_num_for_whole_trajectory();
};




}