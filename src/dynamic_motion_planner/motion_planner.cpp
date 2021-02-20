#include "dynamic_motion_planner/motion_planner.hpp"

namespace hdi_plan {

MotionPlanner::MotionPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
        : nh_(nh),
          pnh_(pnh) {

    // setup
    this->setup();
    this->setup_space();

    // initialization
    ROS_INFO("Initialization");
    this->nearest_neighbors_tree_->add(this->root_);

    // initialization subscribers
    this->sub_quadrotor_state_ = nh_.subscribe("hdi_plan/quadrotor_state", 1, &MotionPlanner::quadrotor_state_callback, this);
    //this->pub_quadrotor_state_ = nh_.advertise<nav_msgs::Odometry>("hdi_plan/quadrotor_debug", 1);
}

MotionPlanner::~MotionPlanner() = default;

double MotionPlanner::distance_function(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) {
    //Todo: implement how to calculate the distance between two rrt nodes here
    return 1.0;
}

void MotionPlanner::setup() {
    Eigen::Vector3d root_position(0,0,0);
    root_ = std::make_shared<RRTNode>(root_position);
    Eigen::Vector3d goal_position(1,1,1);
    goal_ = std::make_shared<RRTNode>(goal_position);

    nearest_neighbors_tree_ = std::make_shared<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTNode>>>();
    nearest_neighbors_tree_->setDistanceFunction([this](const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) { return distance_function(a, b); });
}

void MotionPlanner::setup_space() {
    this->space_ = std::make_shared<ompl::base::SE3StateSpace>();
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    this->space_->setBounds(bounds);
    this->si_ = std::make_shared<ompl::base::SpaceInformation>(this->space_);
}

ompl::base::RealVectorBounds MotionPlanner::get_space_info() {
    return this->space_->getBounds();
}

int MotionPlanner::get_dim() {
    return this->space_->getDimension();
}

int MotionPlanner::get_debug() {
    return this->debug;
}

double MotionPlanner::get_nn_size() {
    return static_cast<double>(this->nearest_neighbors_tree_->size());
}

// not the static function, need to use the space config
std::shared_ptr<RRTNode> MotionPlanner::generate_random_node() {
    Eigen::Vector3d random_position(0.5,0.5,0.5);
    auto random_node = std::make_shared<RRTNode>(random_position);
    return random_node;
}

void MotionPlanner::setup_sub() {
    // initialize subscriber call backs

}

void MotionPlanner::quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    Eigen::Vector3d quadrotor_state(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    this->quadrotor_state_ = quadrotor_state;
    this->debug += 1;
    ROS_INFO("Check if reach the goal region: ");

    ROS_INFO("Now trigger the main loop of dynamic planner");
    //this->pub_quadrotor_state_.publish(msg);
    if (this->get_distance_between_states(quadrotor_state, this->goal_->get_state()) > 0.1) {
        this->solve();
    } else {
        ROS_INFO("Already reach the goal. Will exit.");
        // need to implement how to exit here. And other data collection like: total time. total length of executed trajectory.
    }
}

double MotionPlanner::get_distance_between_states(Eigen::Vector3d state1, Eigen::Vector3d state2) {
    double distance = std::sqrt((state1 - state2).squaredNorm());
    return distance;
}

void MotionPlanner::solve() {
    this->calculateRRG();

    // sample random state
    
    // need to implement how to generate random state here, by adding the info of updating obstacles
    std::shared_ptr<RRTNode> new_node = generate_random_node();

    // find the closest node in the tree
    std::shared_ptr<RRTNode> nearest_node = this->nearest_neighbors_tree_->nearest(new_node);
    double distance = this->get_distance_between_states(new_node->get_state(), nearest_node->get_state());
    if (distance > this->max_distance_) {
        this->saturate(new_node, nearest_node);
    }

    //this->extend();

    //nearest_neighbors_tree_->add(new_node);


}

void MotionPlanner::calculateRRG() {
    double num_of_nodes_in_tree = static_cast<double>(this->nearest_neighbors_tree_->size());
    this->rrg_r_ = std::min(this->max_distance_, this->r_rrt_ * std::pow(log(num_of_nodes_in_tree)/num_of_nodes_in_tree, 1/static_cast<double>(this->dimension_)));
}

void MotionPlanner::saturate(std::shared_ptr<RRTNode> new_node, std::shared_ptr<RRTNode> nearest_node) {
    // move the newnode to distance = sigma to nearest node here
}
}

