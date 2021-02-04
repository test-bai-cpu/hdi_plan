#pragma once

#include <stack>
#include <vector>
#include <Eigen/Dense>

//ompl
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>

//ros
#include <ros/ros.h>

#include "dynamic_motion_planner/node_list.hpp"
#include "dynamic_motion_planner/edge.hpp"
#include "dynamic_motion_planner/quadrotor.hpp"
#include "dynamic_motion_planner/RRT_node.hpp"
#include "dynamic_motion_planner/space.hpp"

namespace hdi_plan {

class MotionPlanner {
public:
    MotionPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~MotionPlanner();

    // debug
    ompl::base::RealVectorBounds get_space_info();
    int get_dim();

private:
    // ros nodes
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // initialize stack for storing points that desire to insert in the future when an obstacle is removed
    std::stack<std::vector<Eigen::Vector3d>> sample_stack;

    // useful rrt nodes
    std::shared_ptr<RRTNode> root;
    std::shared_ptr<RRTNode> goal;

    // space configuration
    ompl::base::SpaceInformationPtr si_;
    std::shared_ptr<ompl::base::SE3StateSpace> space_;
    void setup_space();

    // geometric Near-neighbor Access Tree
    std::shared_ptr<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTNode>>> nearest_neighbors_tree;

    // setup
    void setup();
    static float distance_function(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) ;

    //
    std::shared_ptr<RRTNode> generate_random_node();

};


}