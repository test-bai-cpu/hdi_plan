#pragma once

#include <stack>
#include <queue>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <math.h>

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
    unsigned int get_dim();
    int get_debug();
    double get_nn_size();
    //publisher to debug
    ros::Publisher pub_quadrotor_state_;

    struct node_compare{
        bool operator()(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b){
            double a_min_cost = std::min(a->get_lmc(), a->get_g_cost());
            double b_min_cost = std::min(b->get_lmc(), b->get_g_cost());

            if (a_min_cost == b_min_cost) return a->get_g_cost() > b->get_g_cost();
            return a_min_cost > b_min_cost;
        }
    };
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
    double distance_function(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) ;

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
    bool solve();

    // unitilites
    double get_distance_between_states(Eigen::Vector3d state1, Eigen::Vector3d state2);

    // for calculate the shrinking ball radius
    void calculateRRG();
    double max_distance_{0.}; // the maximum length of a motion to be added to a tree
    double r_rrt_{0.}; // a constant for r-disc rewiring calculations
    double rrg_r_; // current value of the radius used for the neighbors

    // functions in the main loop
    void saturate(std::shared_ptr<RRTNode> random_node, const std::shared_ptr<RRTNode>& nearest_node, double distance) const;
    bool node_in_free_space_check(const std::shared_ptr<RRTNode>& random_node);
    bool edge_in_free_space_check(const std::shared_ptr<RRTNode>& node1, const std::shared_ptr<RRTNode>& node2);
    bool extend(std::shared_ptr<RRTNode> random_node);

    // extend related
    void update_neighbors_list(std::shared_ptr<RRTNode> random_node);
    bool find_best_parent(std::shared_ptr<RRTNode> random_node);

    // cost related members
    double compute_cost(Eigen::Vector3d state1, Eigen::Vector3d state2);
    double combine_cost(double cost1, double cost2);
    bool is_cost_better_than(double cost1, double cost2);

    // rewire
    double epsilon_ = 0;


    // cannot use priority queue since we need to update the key value
    //std::priority_queue<std::shared_ptr<RRTNode>, std::vector<std::shared_ptr<RRTNode>>, node_compare> node_queue1;
    ompl::BinaryHeap<std::shared_ptr<RRTNode>, node_compare> node_queue;
    void rewire_neighbors(std::shared_ptr<RRTNode> random_node);
    void cull_neighbors(std::shared_ptr<RRTNode> random_node);
    void verify_queue(std::shared_ptr<RRTNode> node);
    void reduce_inconsistency();

};


}