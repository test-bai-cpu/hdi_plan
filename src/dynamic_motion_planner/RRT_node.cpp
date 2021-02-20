#include <ros/ros.h>

#include "dynamic_motion_planner/RRT_node.hpp"

namespace hdi_plan {

RRTNode::RRTNode(Eigen::Vector3d state) {
    this->state= state;
}

RRTNode::~RRTNode() {
}

}