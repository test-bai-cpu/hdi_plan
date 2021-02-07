#include "dynamic_motion_planner/edge.hpp"

namespace hdi_plan {

Edge::Edge(const RRTNode& start_node, const RRTNode& end_node) {
    this->start_node = start_node;
    this->end_node = end_node;
}

Edge::~Edge() {
}

float Edge::calculate_dist() {
    return 1.0;
}

}