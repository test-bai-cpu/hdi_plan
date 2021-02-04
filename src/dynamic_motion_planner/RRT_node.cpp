#include <ros/ros.h>

#include "dynamic_motion_planner/RRT_node.hpp"

namespace hdi_plan {

RRTNode::RRTNode(Eigen::Vector3d position) {
    this->position = position;
}

RRTNode::~RRTNode() {
}

}