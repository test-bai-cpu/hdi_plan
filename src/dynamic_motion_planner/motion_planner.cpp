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
    Eigen::Vector3d goal_position(10,10,10);
    goal_ = std::make_shared<RRTNode>(goal_position);

    nearest_neighbors_tree_ = std::make_shared<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTNode>>>();
    nearest_neighbors_tree_->setDistanceFunction([this](const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) { return distance_function(a, b); });
}

void MotionPlanner::setup_space() {
    this->space_ = std::make_shared<ompl::base::SE3StateSpace>();
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
    this->space_->setBounds(bounds);
    this->si_ = std::make_shared<ompl::base::SpaceInformation>(this->space_);
    //this->space_->interpolate()
}

ompl::base::RealVectorBounds MotionPlanner::get_space_info() {
    return this->space_->getBounds();
}

unsigned int MotionPlanner::get_dim() {
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
    //ROS_INFO("Check if reach the goal region: ");

    //ROS_INFO("Now trigger the main loop of dynamic planner");
    //this->pub_quadrotor_state_.publish(msg);
    if (this->get_distance_between_states(quadrotor_state, this->goal_->get_state()) > 0.1) {
        this->solve();
    } else {
        ROS_INFO("Already reach the goal. Will exit.");
        // need to implement how to exit here. And other data collection like: total time. total length of executed trajectory.
    }
}

double MotionPlanner::get_distance_between_states(Eigen::Vector3d state1, Eigen::Vector3d state2) {
    return static_cast<double>(std::sqrt((state1 - state2).squaredNorm()));
}

bool MotionPlanner::solve() {
    this->calculateRRG();

    // sample random state
    
    // need to implement how to generate random state here, by adding the info of updating obstacles
    std::shared_ptr<RRTNode> random_node = generate_random_node();

    // find the closest node in the tree
    std::shared_ptr<RRTNode> nearest_node = this->nearest_neighbors_tree_->nearest(random_node);
    double distance = this->get_distance_between_states(random_node->get_state(), nearest_node->get_state());
    if (distance > this->max_distance_) {
        this->saturate(random_node, nearest_node, distance);
    }

    if (!this->node_in_free_space_check(random_node)) {
        return false;
    }

    if (!this->extend(random_node)) {
        return false;
    }

    this->rewire_neighbors(random_node);
    this->reduce_inconsistency();

}

void MotionPlanner::rewire_neighbors(std::shared_ptr<RRTNode> random_node) {
    this->cull_neighbors(random_node);

    for (auto it = random_node->nr_out.begin(); it != random_node->nr_out.end(); ++it) {
        
    }
}

void MotionPlanner::cull_neighbors(std::shared_ptr<RRTNode> random_node) {
    for (auto it = random_node->nr_out.begin(); it != random_node->nr_out.end(); ++it) {
        std::shared_ptr<RRTNode> neighbor = *it;
        if (this->rrg_r_ < this->compute_cost(random_node->get_state(), neighbor->get_state()) && random_node->parent != neighbor) {
            random_node->nr_out.erase(it);
            for (auto u_it = neighbor->nr_in.begin(); u_it != neighbor->nr_in.end(); ++u_it) {
                if (*u_it == random_node) {
                    neighbor->nr_in.erase(u_it);
                }
            }
        }
    }
}

void MotionPlanner::reduce_inconsistency() {

}

bool MotionPlanner::node_in_free_space_check(const std::shared_ptr<RRTNode>& random_node) {
    return true;
}

bool MotionPlanner::edge_in_free_space_check(const std::shared_ptr<RRTNode>& node1, const std::shared_ptr<RRTNode>& node2) {
    return true;
}

void MotionPlanner::calculateRRG() {
    auto num_of_nodes_in_tree = static_cast<double>(this->nearest_neighbors_tree_->size());
    this->rrg_r_ = std::min(this->max_distance_, this->r_rrt_ * std::pow(log(num_of_nodes_in_tree)/num_of_nodes_in_tree, 1/static_cast<double>(this->dimension_)));
}

void MotionPlanner::saturate(std::shared_ptr<RRTNode> random_node, const std::shared_ptr<RRTNode>& nearest_node, double distance) const {
    // move the random node to distance = sigma to nearest node here, using interpolate
    Eigen::Vector3d random_node_state = random_node->get_state();
    Eigen::Vector3d nearest_node_state = nearest_node->get_state();
    double x = (random_node_state(0) - nearest_node_state(0)) * this->max_distance_ / distance + nearest_node_state(0);
    double y = (random_node_state(1) - nearest_node_state(1)) * this->max_distance_ / distance + nearest_node_state(1);
    double z = (random_node_state(2) - nearest_node_state(2)) * this->max_distance_ / distance + nearest_node_state(2);
    ROS_INFO("Change the position of random node to be closer to nearest node");
    random_node->set_state_by_value(x,y,z);
}

// inserting a new node
bool MotionPlanner::extend(std::shared_ptr<RRTNode> random_node) {
    // v is random_node, u is neighbor
    // 1. find all nodes within shrinking hyperball in the nearest tree, and their distance
    this->update_neighbors_list(random_node);
    if (! this->find_best_parent(random_node)) {
        return false;
    }
    this->nearest_neighbors_tree_->add(random_node);
    random_node->parent->children.push_back(random_node);

    // after connecting the parent, now is updating list of v's neighbors
    for (auto it = random_node->nbh.begin(); it != random_node->nbh.end(); ++it) {
        std::shared_ptr<RRTNode> neighbor = it->first;

        if (this->edge_in_free_space_check(random_node, neighbor)) {
            random_node->n0_out.push_back(neighbor);
            neighbor->nr_in.push_back(random_node);
        }
        if (this->edge_in_free_space_check(neighbor, random_node)) {
            neighbor->nr_out.push_back(random_node);
            random_node->n0_in.push_back(neighbor);
        }
    }
    return true;
}

bool MotionPlanner::find_best_parent(std::shared_ptr<RRTNode> random_node) {
    bool if_find_best_parent = false;
    for (auto it = random_node->nbh.begin(); it != random_node->nbh.end(); ++it) {
        std::shared_ptr<RRTNode> neighbor = it->first;
        // compute cost using this neighbor as a parent
        double inc_cost = this->compute_cost(random_node->get_state(), neighbor->get_state());  // d(v,u)
        double cost = this->combine_cost(neighbor->get_lmc(), inc_cost); // d(v,u) + lmc(u)
        // if lmc(v) > d(v,u) + lmc(u)
        if (this->is_cost_better_than(cost, random_node->get_lmc()) && this->edge_in_free_space_check(random_node, neighbor)) {
            it->second = true;
            // change parent of v to u
            random_node->parent = neighbor;
            // update lmc(v)
            random_node->set_lmc(cost);
            random_node->set_g_cost(cost);
            if_find_best_parent = true;
        }
    }
    return if_find_best_parent;
}

void MotionPlanner::update_neighbors_list(std::shared_ptr<RRTNode> random_node) {
    std::vector<std::shared_ptr<RRTNode>> nbh;
    this->nearest_neighbors_tree_->nearestR(random_node, this->rrg_r_, nbh);

    random_node->nbh.resize(nbh.size());
    // the default bool value of all added nodes are false, the bool value is the feasibility of edge as been tested
    std::transform(nbh.begin(), nbh.end(), random_node->nbh.begin(), [](std::shared_ptr<RRTNode> node) { return std::pair<std::shared_ptr<RRTNode>, bool>(node, false);});
}

double MotionPlanner::compute_cost(Eigen::Vector3d state1, Eigen::Vector3d state2) {
    //Todo: implement how to compute cost by the map here, need more discussion
    return static_cast<double>(std::sqrt((state1 - state2).squaredNorm()));
}

double MotionPlanner::combine_cost(double cost1, double cost2) {
    return cost1 + cost2;
}

bool MotionPlanner::is_cost_better_than(double cost1, double cost2) {
    return cost1 < cost2;
}

}

