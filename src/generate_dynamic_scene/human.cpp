#include "generate_dynamic_scene/human.hpp"

namespace hdi_plan {
Human::Human(Eigen::Vector2d start_position, double start_time)
		: start_position_(start_position), start_time_(start_time) {

}

Human::~Human() = default;

double Human::get_distance(const Eigen::Vector2d& state1, const Eigen::Vector2d& state2) {
	return static_cast<double>(std::sqrt((state1 - state2).squaredNorm()));
}

Eigen::Vector2d Human::predict_path(double node_time) {
	double move_distance = (node_time - this->start_time_) * this->human_velocity_;
	double unit_distance = this->get_distance(this->start_position_, this->second_position_);

	Eigen::Vector2d predict_position(this->start_position_(0), this->start_position_(1));

	predict_position(0) = (this->second_position_(0) - this->start_position_(0)) * move_distance / unit_distance + this->start_position_(0);
	predict_position(1) = (this->second_position_(1) - this->start_position_(1)) * move_distance / unit_distance + this->start_position_(1);

	return predict_position;
}

bool Human::check_if_node_inside_human(const std::shared_ptr<RRTNode>& node) {
	Eigen::Vector2d node_position(node->get_state()(0), node->get_state()(1));
	double node_time = node->get_time();

	if (node_time < this->start_time_) {
		return false;
	}

	if (node->get_state()(2)>this->human_height_) {
		return false;
	}

	if (!this->if_move_) {
		return this->get_distance(this->start_position_, node_position)<this->human_block_distance_;
	}

	Eigen::Vector2d predict_position = this->predict_path(node_time);
	return this->get_distance(predict_position, node_position)<this->human_block_distance_;
}

}

