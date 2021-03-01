#include <ros/ros.h>

#include "dynamic_motion_planner/RRT_node.hpp"

namespace hdi_plan {

RRTNode::RRTNode(Eigen::Vector3d state) {
    this->state_= state;
    double inf = std::numeric_limits<double>::infinity();
    this->lmc_ = inf;
    this->g_cost_ = inf;
}

RRTNode::~RRTNode() {
}

}