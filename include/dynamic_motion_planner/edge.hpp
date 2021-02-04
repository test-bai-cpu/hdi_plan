#pragma once

#include "dynamic_motion_planner/RRT_node.hpp"

namespace hdi_plan {
class Edge{
public:
    Edge();
    ~Edge();
    float calculate_dist();

private:
    RRTNode start_node;
    RRTNode end_node;
    float dist;
    float dist_original;
    float w_dist;
};



}