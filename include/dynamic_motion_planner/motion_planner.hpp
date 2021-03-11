#pragma once

#include <stack>
#include <queue>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <time.h>
//ompl
#include "ompl/datastructures/NearestNeighbors.h"
#include <ompl/datastructures/BinaryHeap.h>
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>


//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

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
    double get_nn_size();

private:
    // debug
    int debug{0};
    int count{0};

    // ros nodes
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // initialize stack for storing points that desire to insert in the future when an obstacle is removed
    std::stack<std::vector<Eigen::Vector3d>> sample_stack;

    // useful rrt nodes
    std::shared_ptr<RRTNode> start_;
    std::shared_ptr<RRTNode> goal_;
    std::shared_ptr<RRTNode> quadrotor_;

    // space configuration
    ompl::base::SpaceInformationPtr si_;
    std::shared_ptr<ompl::base::SE3StateSpace> space_;
    void setup_space();
    int dimension_{3};

    // geometric Near-neighbor Access Tree
    std::shared_ptr<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTNode>>> nearest_neighbors_tree_;

    // setup
    void setup();
    double distance_function(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) ;
    void set_to_start_position();
    bool position_ready{false};

    //
    std::shared_ptr<RRTNode> generate_random_node();

    // quadrotor related
    Eigen::Vector3d quadrotor_state_;

    // subscriber
    ros::Subscriber sub_quadrotor_state_;
    void quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg);

    // publisher
    ros::Publisher pub_solution_path_;

    // main loop
    bool solve();

    // for calculate the shrinking ball radius
    void calculateRRG();
    double max_distance_{0.5}; // the maximum length of a motion to be added to a tree
    double r_rrt_{1}; // a constant for r-disc rewiring calculations
    double rrg_r_{0.5}; // current value of the radius used for the neighbors

    // functions in the main loop
    void saturate(std::shared_ptr<RRTNode> random_node, const std::shared_ptr<RRTNode>& nearest_node, double distance) const;
    bool node_in_free_space_check(const std::shared_ptr<RRTNode>& random_node);
    bool edge_in_free_space_check(const std::shared_ptr<RRTNode>& node1, const std::shared_ptr<RRTNode>& node2);
    bool edge_in_free_space_check_using_state(const Eigen::Vector3d& state1, const Eigen::Vector3d& state2);
    bool extend(std::shared_ptr<RRTNode> random_node);

    // extend related
    void update_neighbors_list(std::shared_ptr<RRTNode> random_node);
    bool find_best_parent(std::shared_ptr<RRTNode> random_node);

    // cost related members
    double compute_cost(const Eigen::Vector3d& state1, const Eigen::Vector3d& state2);
    double combine_cost(double cost1, double cost2);
    bool is_cost_better_than(double cost1, double cost2);

    // rewire
    double epsilon_{0.5};

    // cannot use priority queue since we need to update the key value
    //std::priority_queue<std::shared_ptr<RRTNode>, std::vector<std::shared_ptr<RRTNode>>, node_compare> node_queue1;
    ompl::BinaryHeap<std::shared_ptr<RRTNode>, RRTNode::node_compare> node_queue;
    void rewire_neighbors(std::shared_ptr<RRTNode> random_node);
    void cull_neighbors(std::shared_ptr<RRTNode> random_node);
    void verify_queue(std::shared_ptr<RRTNode> node);
    void reduce_inconsistency();
    void reduce_inconsistency_for_env_update();
    void update_lmc(std::shared_ptr<RRTNode> node);

    // find the solution path
    std::vector<Eigen::Vector3d> solution_path;
    bool update_solution_path();
    void publish_solution_path();
};


}