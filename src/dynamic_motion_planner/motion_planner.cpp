#include "dynamic_motion_planner/motion_planner.hpp"

namespace hdi_plan {

MotionPlanner::MotionPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
        : nh_(nh),
          pnh_(pnh) {

    // setup
    this->setup();
    this->setup_space();
    srand((unsigned)time(NULL));

    // wait until the gazebo and unity are loaded
    ros::Duration(7.0).sleep();

    // initialization
    ROS_INFO("Initialization");
    this->nearest_neighbors_tree_->add(this->goal_);

    // initialization subscribers
    this->sub_quadrotor_state_ = nh_.subscribe("hdi_plan/quadrotor_state", 1, &MotionPlanner::quadrotor_state_callback, this);
    this->sub_obstacle_info_ = nh_.subscribe("hdi_plan/obstacle_info_topic", 1, &MotionPlanner::obstacle_info_callback, this);
    this->pub_solution_path_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
}

MotionPlanner::~MotionPlanner() = default;

double MotionPlanner::distance_function(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) {
    return this->compute_cost(a->get_state(), b->get_state());
}

void MotionPlanner::setup() {
    Eigen::Vector3d start_position(0,0,0);
    this->start_ = std::make_shared<RRTNode>(start_position);
    //this->quadrotor_ = std::make_shared<RRTNode>(start_position);
    Eigen::Vector3d goal_position(2.5,2.5,2.5);
    this->goal_ = std::make_shared<RRTNode>(goal_position);
    this->goal_->set_lmc(0);
    this->goal_->set_g_cost(0);

    this->nearest_neighbors_tree_ = std::make_shared<ompl::NearestNeighborsGNAT<std::shared_ptr<RRTNode>>>();
    this->nearest_neighbors_tree_->setDistanceFunction([this](const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) { return distance_function(a, b); });
}

void MotionPlanner::set_to_start_position() {
    Eigen::Vector3d start_position = this->start_->get_state();

    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = start_position(0);
    msg.pose.position.y = start_position(1);
    msg.pose.position.z = start_position(2);
    this->pub_solution_path_.publish(msg);

    this->position_ready = true;
}

void MotionPlanner::setup_space() {
    this->space_ = std::make_shared<ompl::base::SE3StateSpace>();
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    this->space_->setBounds(bounds);
    this->si_ = std::make_shared<ompl::base::SpaceInformation>(this->space_);
    //this->space_->interpolate()
}

ompl::base::RealVectorBounds MotionPlanner::get_space_info() {
    return this->space_->getBounds();
}

double MotionPlanner::get_nn_size() {
    return static_cast<double>(this->nearest_neighbors_tree_->size());
}

// not the static function, need to use the space config
std::shared_ptr<RRTNode> MotionPlanner::generate_random_node() {
    double lower_bound = 0;
    double upper_bound = 3;
    double x = lower_bound + (rand()/double(RAND_MAX)*(upper_bound - lower_bound));
    double y = lower_bound + (rand()/double(RAND_MAX)*(upper_bound - lower_bound));
    double z = lower_bound + (rand()/double(RAND_MAX)*(upper_bound - lower_bound));
    Eigen::Vector3d random_position(x,y,z);
    auto random_node = std::make_shared<RRTNode>(random_position);
    std::cout << "Generate random node position is: " << x << " " << y << " " << z << std::endl;
    return random_node;
}

void MotionPlanner::quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    //ros::Duration(0.5).sleep();
    Eigen::Vector3d quadrotor_state(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    this->quadrotor_state_ = quadrotor_state;
    this->quadrotor_ = std::make_shared<RRTNode>(quadrotor_state);
    if (this->compute_cost(quadrotor_state, this->goal_->get_state()) > 0.1) {
        this->solve();
    } else {
        ROS_INFO("Already reach the goal. Will exit.");
        this->sub_quadrotor_state_.shutdown();
    }
}

void MotionPlanner::obstacle_info_callback(const hdi_plan::obstacle_info::ConstPtr &msg) {
	std::string obstacle_name = msg->name;
	Obstacle_type obstacle_type = msg->type;
	bool obstacle_operation = msg->operation;
	Eigen::Vector3d obstacle_position(msg->position.x, msg->position.y, msg->position.z);
	auto obstacle = std::make_shared<Obstacle>(obstacle_name, obstacle_type, obstacle_operation, obstacle_position);

	this->obstacle_update_info_list.push_back(obstacle);
}

bool MotionPlanner::solve() {
    this->count += 1;
    std::cout << "The iteration number is: " << this->count << std::endl;
    if(!position_ready) {
        this->set_to_start_position();
    }

    this->calculateRRG();

    this->update_obstacle();

    // sample random state
    // need to implement how to generate random state here, by adding the info of updating obstacles
    std::shared_ptr<RRTNode> random_node = generate_random_node();

    // find the closest node in the tree
    std::shared_ptr<RRTNode> nearest_node = this->nearest_neighbors_tree_->nearest(random_node);

    double distance = this->compute_cost(random_node->get_state(), nearest_node->get_state());

    if (distance > this->max_distance_) {
        this->saturate(random_node, nearest_node, distance);
    }

    // Todo: Question: even when no random_node added, do we need to publish the solution path? YES, because when the environment changes, the solution path will change as well.
    if (!this->extend(random_node)) {
        return false;
    }

    this->rewire_neighbors(random_node);
    this->reduce_inconsistency();

    if (this->update_solution_path()) {
        this->publish_solution_path();
    }
    return true;
}

void MotionPlanner::update_obstacle() {
	bool add_obstacle = false;
	bool remove_obstacle = false;

	for (auto it = this->obstacle_update_info_list.begin(); it != this->obstacle_update_info_list.end(); ++it) {
		std::shared_ptr<Obstacle> obstacle = *it;
		if (!obstacle->get_operation()) {
			this->obstacle_map.erase(obstacle->get_name());
			this->remove_obstacle(obstacle);
			remove_obstacle = true;
		}
	}
	if (remove_obstacle) {
		this->reduce_inconsistency_for_env_update();
	}

	for (auto it = this->obstacle_update_info_list.begin(); it != this->obstacle_update_info_list.end(); ++it) {
		std::shared_ptr<Obstacle> obstacle = *it;
		if (obstacle->get_operation()) {
			this->obstacle_map[obstacle->get_name()] = obstacle;
			this->add_obstacle(obstacle);
			add_obstacle = true;
		}
	}
	if (add_obstacle) {
		this->propogate_descendants();
		this->verify_queue(v_bot);
		this->reduce_inconsistency_for_env_update();
	}

	this->obstacle_update_info_list.clear();
}

void MotionPlanner::remove_obstacle(const std::shared_ptr<Obstacle>& obstacle) {
	std::vector<std::shared_ptr<RRTNode>> nodes_list;
	this->nearest_neighbors_tree_->list(nodes_list);

	for (auto it = nodes_list.begin(); it != nodes_list.end(); ++it) {
		std::shared_ptr<RRTNode> node = *it;
		if (!check_if_node_inside_obstacle(obstacle, node) || check_if_node_inside_all_obstalces(node)) {
			continue;
		}
		this->update_lmc(node);
		if (node->get_lmc() != node->get_g_cost()) {
			this->verify_queue(node);
		}
	}
}

bool MotionPlanner::check_if_node_inside_obstacle(const std::shared_ptr<Obstacle>& obstacle, const std::shared_ptr<RRTNode>& node) {
	return false;
}

void MotionPlanner::add_obstacle(const std::shared_ptr<Obstacle>& obstacle) {
	std::vector<std::shared_ptr<RRTNode>> nodes_list;
	this->nearest_neighbors_tree_->list(nodes_list);

	for (auto it = nodes_list.begin(); it != nodes_list.end(); ++it) {
		std::shared_ptr<RRTNode> node = *it;
		if (!check_if_node_inside_obstacle(obstacle, node)) {
			continue;
		}
		this->verify_orphan(node);
	}
}

void MotionPlanner::verify_orphan(std::shared_ptr<RRTNode> node) {
	if (node->handle != nullptr) {
		this->node_queue.remove(node->handle);
		node->handle = nullptr;
	}
	this->orphan_node_list.push_back(node);
}

void MotionPlanner::propogate_descendants() {

}

bool MotionPlanner::update_solution_path() {
    bool if_find_solution = false;

    std::shared_ptr<RRTNode> nearest_node_of_quadrotor = this->nearest_neighbors_tree_->nearest(this->quadrotor_);

    double cost_to_nearest_node = this->compute_cost(nearest_node_of_quadrotor->get_state(), this->quadrotor_->get_state());

    std::cout << "The distance from current position to nearest node: " << cost_to_nearest_node << std::endl;
    if (cost_to_nearest_node > this->max_distance_) {
        return if_find_solution; // return false, fail to find solution
    }
    if (!this->edge_in_free_space_check(this->quadrotor_, nearest_node_of_quadrotor)) {
        return if_find_solution; // return false, fail to find solution
    }

    this->solution_path.clear();
    std::shared_ptr<RRTNode> intermediate_node = nearest_node_of_quadrotor;

    while (intermediate_node == this->goal_ || intermediate_node->parent != nullptr) {
        this->solution_path.push_back(intermediate_node->get_state());
        if (intermediate_node == this->goal_) {
            if_find_solution = true;
            break;
        }
        intermediate_node = intermediate_node->parent;
    }
    return if_find_solution;
}

void MotionPlanner::publish_solution_path() {
    ROS_INFO("Found the solution path");
    Eigen::Vector3d next_position;
    //Eigen::Vector3d next_position = this->solution_path.at(1);
    for (auto it = this->solution_path.begin(); it != this->solution_path.end(); ++it) {
        next_position = *it;
        std::cout << "The solution path is: " << next_position(0) << " " << next_position(1) << " " << next_position(2) << std::endl;
        if (this->compute_cost(next_position, this->quadrotor_->get_state()) > 0.1) {
            break;
        }
    }

    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = next_position(0);
    msg.pose.position.y = next_position(1);
    msg.pose.position.z = next_position(2);

    ROS_INFO("Sending solution path to the quadrotor");
    std::cout << "The command pose is: " << next_position(0) << " " << next_position(1) << " " << next_position(2) << std::endl;
    this->pub_solution_path_.publish(msg);
}

void MotionPlanner::rewire_neighbors(std::shared_ptr<RRTNode> random_node) {
    ROS_INFO("In rewire now");
    // if this step is for random_node, then Nr of v is empty, fo cull_neighbor will not cull any nodes
    if (random_node->get_g_cost() - random_node->get_lmc() <= this->epsilon_) {
        return;
    }
    this->cull_neighbors(random_node);
    std::vector<std::shared_ptr<RRTNode>> n_in;
    n_in.insert(n_in.end(), random_node->n0_in.begin(), random_node->n0_in.end());
    n_in.insert(n_in.end(), random_node->nr_in.begin(), random_node->nr_in.end());
    for (auto it = n_in.begin(); it != n_in.end(); ++it) {
        std::shared_ptr<RRTNode> neighbor = *it;
        if (random_node->parent == neighbor) {
            continue;
        }
        double new_lmc_if_choose_random_node_as_parent = random_node->get_lmc()+this->compute_cost(neighbor->get_state(), random_node->get_state());
        if (neighbor->parent != this->goal_ && this->is_cost_better_than(new_lmc_if_choose_random_node_as_parent, neighbor->get_lmc())) {
            neighbor->set_lmc(new_lmc_if_choose_random_node_as_parent);
            neighbor->parent = random_node;
        }
        if (neighbor->get_g_cost() - neighbor->get_lmc() > this->epsilon_) {
            ROS_INFO("In rewire: add to the priority queue");
            this->verify_queue(neighbor);
        }
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
                    break;
                }
            }
            break;
        }
    }
}

void MotionPlanner::verify_queue(std::shared_ptr<RRTNode> node) {
    if (node->handle != nullptr) {
        this->node_queue.update(node->handle);
    } else {
        node->handle = this->node_queue.insert(node);
    }
}

void MotionPlanner::reduce_inconsistency() {
    ROS_INFO("In reduce inconsistency");
    while (!this->node_queue.empty()) {
        std::shared_ptr<RRTNode> min = this->node_queue.top()->data;
        this->node_queue.pop();
        min->handle = nullptr;

        if (min->get_g_cost() - min->get_lmc() > this->epsilon_) {
            this->update_lmc(min);
            this->rewire_neighbors(min);
        }

        min->set_g_cost(min->get_lmc());
    }

    while (!this->node_queue.empty()) {
        this->node_queue.top()->data->handle = nullptr;
        this->node_queue.pop();
    }
    this->node_queue.clear();
}

void MotionPlanner::reduce_inconsistency_for_env_update() {
    ROS_INFO("In reduce inconsistency for env update");
    //Todo: have not add the part of keyless, Q, vbot condition check
    while (!this->node_queue.empty()) {
        std::shared_ptr<RRTNode> min = this->node_queue.top()->data;
        this->node_queue.pop();
        min->handle = nullptr;

        if (min->get_g_cost() - min->get_lmc() > this->epsilon_) {
            this->update_lmc(min);
            this->rewire_neighbors(min);
        }

        min->set_g_cost(min->get_lmc());
    }

    while (!this->node_queue.empty()) {
        this->node_queue.top()->data->handle = nullptr;
        this->node_queue.pop();
    }
    this->node_queue.clear();
}

// find the best parent in N+(v)
void MotionPlanner::update_lmc(std::shared_ptr<RRTNode> node) {
    this->cull_neighbors(node);

    std::vector<std::shared_ptr<RRTNode>> n_out;
    n_out.insert(n_out.end(), node->n0_out.begin(), node->n0_out.end());
    n_out.insert(n_out.end(), node->nr_out.begin(), node->nr_out.end());

    for (auto it = n_out.begin(); it != n_out.end(); ++it) {
        std::shared_ptr<RRTNode> neighbor = *it;

		bool is_orphan = false;
		for (auto orphan = this->orphan_node_list.begin(); orphan != this->orphan_node_list.end(); ++orphan) {
			if (*orphan == neighbor) {
				is_orphan = true;
				break;
			}
		}

        if (is_orphan || neighbor->parent == node) {
            continue;
        }
        double inc_cost = this->compute_cost(node->get_state(), neighbor->get_state());  // d(v,u)
        double cost = this->combine_cost(neighbor->get_lmc(), inc_cost); // d(v,u) + lmc(u)
        // if lmc(v) > d(v,u) + lmc(u)
        if (node->parent != this->goal_ && this->is_cost_better_than(cost, node->get_lmc()) && this->edge_in_free_space_check(node, neighbor)) {
            // change parent of v to u
            node->parent = neighbor;
            // update lmc(v)
            node->set_lmc(cost);
        }
    }
}

bool MotionPlanner::node_in_free_space_check(const std::shared_ptr<RRTNode>& random_node) {
    return true;
}

bool MotionPlanner::edge_in_free_space_check(const std::shared_ptr<RRTNode>& node1, const std::shared_ptr<RRTNode>& node2) {
    return true;
}

bool MotionPlanner::edge_in_free_space_check_using_state(const Eigen::Vector3d& state1, const Eigen::Vector3d& state2) {
    return true;
}

void MotionPlanner::calculateRRG() {
    auto num_of_nodes_in_tree = static_cast<double>(this->nearest_neighbors_tree_->size());
    this->rrg_r_ = std::min(this->max_distance_, this->r_rrt_ * std::pow(log(num_of_nodes_in_tree)/num_of_nodes_in_tree, 1/static_cast<double>(this->dimension_)));
    if (this->rrg_r_ == 0) {
        this->rrg_r_ = this->max_distance_ + 0.1;
    }
    std::cout << "Tree size is: " << num_of_nodes_in_tree << std::endl;
    std::cout << "The rrg_r_ is " << this->rrg_r_ << std::endl;
}

void MotionPlanner::saturate(std::shared_ptr<RRTNode> random_node, const std::shared_ptr<RRTNode>& nearest_node, double distance) const {
    // move the random node to distance = sigma to nearest node here, using interpolate
    Eigen::Vector3d random_node_state = random_node->get_state();
    Eigen::Vector3d nearest_node_state = nearest_node->get_state();
    double x = (random_node_state(0) - nearest_node_state(0)) * this->max_distance_ / distance + nearest_node_state(0);
    double y = (random_node_state(1) - nearest_node_state(1)) * this->max_distance_ / distance + nearest_node_state(1);
    double z = (random_node_state(2) - nearest_node_state(2)) * this->max_distance_ / distance + nearest_node_state(2);
    //ROS_INFO("Change the position of random node to be closer to nearest node");
    std::cout << "Saturate, new position is: " << x << " " << y << " " << z << std::endl;
    random_node->set_state_by_value(x,y,z);
}

// inserting a new node
bool MotionPlanner::extend(std::shared_ptr<RRTNode> random_node) {
    ROS_INFO("In extend now");
    if (!this->node_in_free_space_check(random_node)) {
        return false;
    }
    // v is random_node, u is neighbor
    // 1. find all nodes within shrinking hyperball in the nearest tree, and their distance
    this->update_neighbors_list(random_node);
    if (! this->find_best_parent(random_node)) {
        ROS_INFO("### Cannot find best parent for this random node.");
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

double MotionPlanner::compute_cost(const Eigen::Vector3d& state1, const Eigen::Vector3d& state2) {
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
