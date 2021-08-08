#include <ros/ros.h>

#include "dynamic_motion_planner/RRT_node.hpp"

namespace hdi_plan {
RRTNode::RRTNode(Eigen::Vector3d state, double time_from_start) {
    this->state_= state;
    double inf = std::numeric_limits<double>::infinity();
    this->lmc_ = inf;
    this->g_cost_ = inf;
    this->time_ = time_from_start;
    this->handle = nullptr;
    this->parent = nullptr;
}

RRTNode::~RRTNode() = default;

void RRTNode::add_to_neighbor_edge_list(double edge) {
	this->neighbor_edge_list_.push_back(edge);
}

void RRTNode::update_neighbor_edge_list(int neighbor_index, double edge) {
	this->neighbor_edge_list_.at(neighbor_index) = edge;
}

double RRTNode::get_neighbor_edge(int neighbor_index) {
	return this->neighbor_edge_list_.at(neighbor_index);
}

}