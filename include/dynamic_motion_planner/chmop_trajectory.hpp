#pragma once

#include <math.h>
#include <vector>
#include <Eigen/Dense>

#include "utils/utility_functions.hpp"

namespace hdi_plan {
class ChmopTrajectory {
public:
	ChmopTrajectory(const std::vector<Eigen::Vector3d>& trajectory_points);
	ChmopTrajectory(Eigen::Vector3d start_point, Eigen::Vector3d end_point);
	~ChmopTrajectory();

	int get_num_points() const {
		return this->num_points_;
	};

	int get_num_points_diff() const {
		return this->num_points_diff_;
	};

	int get_num_points_free() const {
		return this->num_points_free_;
	};

	int get_start_extra() const {
		return this->start_extra_;
	};

	int get_end_extra() const {
		return this->end_extra_;
	};

	int get_start_index() const {
		return this->start_index_;
	};

	int get_end_index() const {
		return this->end_index_;
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
	double duration_diff_;
	int num_points_;
	int num_points_diff_;
	int num_points_free_;
	double discretization_{0.01}; // every interval is 0.01s
	double speed_{1.0};
	std::vector<int> group_number_;
	std::vector<int> group_number_add_;

	int start_index_origin;
	int end_index_origin_;
	int start_index_;
	int end_index_;
	int start_extra_{0};
	int end_extra_{0};
	void calculate_duration_and_points_num_for_full_trajectory();
	void update_trajectory_for_diff();
};




}