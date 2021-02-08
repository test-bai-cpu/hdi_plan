#pragma once

#include <stack>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <math.h>

//ompl
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>

//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

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
    int get_debug();
    double get_nn_size();
    //publisher to debug
    ros::Publisher pub_quadrotor_state_;


private:
    // debug
    int debug{0};

    // ros nodes
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // initialize stack for storing points that desire to insert in the future when an obstacle is removed
    std::stack<std::vector<Eigen::Vector3d>> sample_stack;

    // useful rrt nodes
    std::shared_ptr<RRTNode> root_;
    std::shared_ptr<RRTNode> goal_;

    // space configuration
    ompl::base::SpaceInformationPtr si_;
    std::shared_ptr<ompl::base::SE3StateSpace> space_;
    void setup_space();
    int dimension_{3};

    // geometric Near-neighbor Access Tree
    std::shared_ptr<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTNode>>> nearest_neighbors_tree_;

    // setup
    void setup();
    static double distance_function(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) ;

    //
    std::shared_ptr<RRTNode> generate_random_node();

    // quadrotor related
    Eigen::Vector3d quadrotor_state_;

    // subscriber
    void setup_sub();
    ros::Subscriber sub_quadrotor_state_;
    void quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg);

    // publisher

    // main loop
    void solve();

    // unitilites
    double get_distance_between_states(Eigen::Vector3d state1, Eigen::Vector3d state2);

    // for calculate the shrinking ball radius
    void calculateRRG();
    double max_distance_{0.}; // the maximum length of a motion to be added to a tree
    double r_rrt_{0.}; // a constant for r-disc rewiring calculations
    double rrg_r_; // current value of the radius used for the neighbors


};


}