#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <queue>
#include <memory>

#include "dynamic_motion_planner/RRT_node.hpp"

namespace hdi_plan {

class Human {
public:
	Human(Eigen::Vector2d start_position, int human_id, double start_time);
	~Human();

	Eigen::Vector2d predict_path(double node_time);
	bool check_if_node_inside_human(const std::shared_ptr<RRTNode>& node);
	double get_distance(const Eigen::Vector2d& state1, const Eigen::Vector2d& state2);

	int get_human_id() const {
		return this->human_id_;
	};

	void set_human_id(int human_id) {
		this->human_id_ = human_id;
	};

	bool get_if_move() const {
		return this->if_move_;
	}

	void set_if_move(bool if_move) {
		this->if_move_ = if_move;
	};

	bool get_if_add() const {
		return this->if_add_;
	}

	void set_if_add(bool if_add) {
		this->if_add_ = if_add;
	};

	void set_start_position(Eigen::Vector2d start_position) {
		this->start_position_ = start_position;
	};

	void set_second_position(Eigen::Vector2d second_position) {
		this->second_position_ = second_position;
	};

	double get_human_block_distance() {
		return this->human_block_distance_;
	}

	double human_height_{2};
	double human_velocity_{1};
	

private:
	Eigen::Vector2d start_position_;
	Eigen::Vector2d second_position_;
	bool if_move_{false};
	int human_id_{0};
	double start_time_;
	bool if_add_{false};
	double human_block_distance_{3};
};

}