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
    this->nearest_neighbors_tree->add(this->root);

    // sample a node
    std::shared_ptr<RRTNode> new_node = generate_random_node();
    nearest_neighbors_tree->add(new_node);
}

MotionPlanner::~MotionPlanner() = default;

float MotionPlanner::distance_function(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) {
    //Todo: implement how to calculate the distance between two rrt nodes here
    return 1.0;
}

void MotionPlanner::setup() {
    Eigen::Vector3d root_position{0,0,0};
    root = std::make_shared<RRTNode>(root_position);
    Eigen::Vector3d goal_position{1,1,1};
    goal = std::make_shared<RRTNode>(goal_position);

    nearest_neighbors_tree = std::make_shared<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTNode>>>();
    nearest_neighbors_tree->setDistanceFunction([this](const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) { return distance_function(a, b); });
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

// not the static function, need to use the space config
std::shared_ptr<RRTNode> MotionPlanner::generate_random_node() {
    Eigen::Vector3d random_position{0,0,0};
    auto random_node = std::make_shared<RRTNode>(random_position);
    return random_node;
}

}
