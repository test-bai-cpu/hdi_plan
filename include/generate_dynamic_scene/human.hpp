#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <queue>
#include <memory>

#include "dynamic_motion_planner/RRT_node.hpp"

namespace hdi_plan {

class Human {
public:
	Human(Eigen::Vector2d start_position, double start_time);
	~Human();

	Eigen::Vector2d predict_path(double node_time);
	bool check_if_node_inside_human(const std::shared_ptr<RRTNode>& node);
	double get_distance(const Eigen::Vector2d& state1, const Eigen::Vector2d& state2);

	double human_height_{2};
	double human_velocity_{1};
	Eigen::Vector2d start_position_;
	Eigen::Vector2d second_position_;
	double start_time_;
	double human_block_distance_{3};
	bool if_move_{false};
private:

};

}