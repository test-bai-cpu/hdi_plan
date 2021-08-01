#include "dynamic_motion_planner/motion_planner.hpp"

namespace hdi_plan {

MotionPlanner::MotionPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
        : nh_(nh),
          pnh_(pnh) {
	// load parameters
	if (!this->load_params()) {
		ROS_WARN("[%s] Could not load all parameters in motion planner.",
				 this->pnh_.getNamespace().c_str());
	} else {
		ROS_INFO("[%s] Loaded all parameters in motion planner.", this->pnh_.getNamespace().c_str());
	}

    // setup
    this->setup();
    this->setup_space();
    srand((unsigned)time(NULL));

    // wait until the gazebo and unity are loaded
    ros::Duration(20.0).sleep();

    // initialization
    ROS_INFO("Initialization");
    this->nearest_neighbors_tree_->add(this->goal_);
	this->start_time_ = ros::Time::now();
	//this->initiate_obstacles();

    // initialization subscribers
    this->sub_quadrotor_state_ = nh_.subscribe("hdi_plan/quadrotor_state", 1, &MotionPlanner::quadrotor_state_callback, this);
	//this->sub_goal_point_ = nh_.subscribe("hdi_plan/goal_point", 1, &MotionPlanner::goal_point_callback, this);
    this->sub_obstacle_info_ = nh_.subscribe("hdi_plan/obstacle_info_topic", 1, &MotionPlanner::obstacle_info_callback, this);
    this->sub_human_movement_ = nh_.subscribe("hdi_plan/human_movement", 1, &MotionPlanner::human_movement_callback, this);
	this->pub_optimized_path_ = nh_.advertise<hdi_plan::point_array>("hdi_plan/full_trajectory", 1);
	this->pub_go_to_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1);
	this->pub_max_velocity_ = nh_.advertise<std_msgs::Float64>("autopilot/max_velocity", 1);
	this->pub_get_new_path_ = nh_.advertise<std_msgs::Bool>("hdi_plan/get_new_path", 1);
	this->pub_node_pos_ = nh_.advertise<geometry_msgs::Point>("hdi_plan/node_position", 1);
	this->pub_blocked_node_pos_ = nh_.advertise<geometry_msgs::Point>("hdi_plan/blocked_node_position", 1);

	// not use autopilot to pub
	this->pub_solution_path_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

	std::string file_name = "solve_time.txt";
	this->solve_time_path_file.open(file_name);
}

MotionPlanner::~MotionPlanner() = default;

void MotionPlanner::check_params() {
	std::cout << "total_planning_time: " << this->total_plan_time_ << std::endl;
	std::cout << "quadrotor_radius: " << this->quadrotor_radius_ << std::endl;
	std::cout << "quadrotor_speed: " << this->quadrotor_speed_ << std::endl;
	std::cout << "max_distance: " << this->max_distance_ << std::endl;
	std::cout << "r_rrt: " << this->r_rrt_ << std::endl;
	std::cout << "rrg_r: " << this->rrg_r_ << std::endl;
	std::cout << "epsilon: " << this->epsilon_ << std::endl;

	std::cout << "collision_threshold: " << this->collision_threshold_ << std::endl;
	std::cout << "planning_time_limit: " << this->planning_time_limit_ << std::endl;
	std::cout << "max_iterations: " << this->max_iterations_ << std::endl;
	std::cout << "max_iterations_after_collision_free: " << this->max_iterations_after_collision_free_ << std::endl;

	std::cout << "learning_rate: " << this->learning_rate_ << std::endl;
	std::cout << "obstacle_cost_weight: " << this->obstacle_cost_weight_ << std::endl;
	std::cout << "dynamic_obstacle_cost_weight: " << this->dynamic_obstacle_cost_weight_ << std::endl;
	std::cout << "dynamic_collision_factor: " << this->dynamic_collision_factor_ << std::endl;
	std::cout << "smoothness_cost_weight: " << this->smoothness_cost_weight_ << std::endl;

	std::cout << "smoothness_cost_velocity: " << this->smoothness_cost_velocity_ << std::endl;
	std::cout << "smoothness_cost_acceleration: " << this->smoothness_cost_acceleration_ << std::endl;
	std::cout << "smoothness_cost_jerk: " << this->smoothness_cost_jerk_ << std::endl;
	std::cout << "ridge_factor: " << this->ridge_factor_ << std::endl;
	std::cout << "min_clearence: " << this->min_clearence_ << std::endl;
	std::cout << "joint_update_limit: " << this->joint_update_limit_ << std::endl;

	std::cout << "discretization: " << this->discretization_ << std::endl;
}

bool MotionPlanner::load_params() {
	//this->pnh_.getParam("param_name", this->param);
	this->pnh_.getParam("total_planning_time", this->total_plan_time_);
	this->pnh_.getParam("quadrotor_radius", this->quadrotor_radius_);
	this->pnh_.getParam("quadrotor_speed", this->quadrotor_speed_);
	this->pnh_.getParam("max_distance", this->max_distance_);
	this->pnh_.getParam("r_rrt", this->r_rrt_);
	this->pnh_.getParam("rrg_r", this->rrg_r_);
	this->pnh_.getParam("epsilon", this->epsilon_);

	this->pnh_.getParam("collision_threshold", this->collision_threshold_);
	this->pnh_.getParam("planning_time_limit", this->planning_time_limit_);
	this->pnh_.getParam("max_iterations", this->max_iterations_);
	this->pnh_.getParam("max_iterations_after_collision_free", this->max_iterations_after_collision_free_);
	this->pnh_.getParam("learning_rate", this->learning_rate_);
	this->pnh_.getParam("obstacle_cost_weight", this->obstacle_cost_weight_);
	this->pnh_.getParam("dynamic_obstacle_cost_weight", this->dynamic_obstacle_cost_weight_);
	this->pnh_.getParam("dynamic_collision_factor", this->dynamic_collision_factor_);
	this->pnh_.getParam("smoothness_cost_weight", this->smoothness_cost_weight_);
	this->pnh_.getParam("smoothness_cost_velocity", this->smoothness_cost_velocity_);
	this->pnh_.getParam("smoothness_cost_acceleration", this->smoothness_cost_acceleration_);
	this->pnh_.getParam("smoothness_cost_jerk", this->smoothness_cost_jerk_);
	this->pnh_.getParam("ridge_factor", this->ridge_factor_);
	this->pnh_.getParam("min_clearence", this->min_clearence_);
	this->pnh_.getParam("joint_update_limit", this->joint_update_limit_);
	this->pnh_.getParam("discretization", this->discretization_);

	return true;
}

void MotionPlanner::initiate_obstacles() {
	std::cout << "Initiate obstacles, add static obstacles." << std::endl;

	// oil tank
	Eigen::Vector3d obstacle_position(-9, 27, 5);
	auto obstacle = std::make_shared<Obstacle>("oil_tank", Obstacle_type::tank_1, true, 15.0, obstacle_position);
	std::cout << "Add this static obstacle: " << obstacle->get_position() << obstacle->get_size() << obstacle->get_name() << obstacle->get_type() << std::endl;
	this->obstacle_map[obstacle->get_name()] = obstacle;

	// red house
	Eigen::Vector3d obstacle_position2(11, 17, 5);
	auto obstacle2 = std::make_shared<Obstacle>("red_house", Obstacle_type::sphere, true, 15.0, obstacle_position2);
	std::cout << "Add this static obstacle: " << obstacle2->get_position() << obstacle2->get_size() << obstacle2->get_name() << obstacle2->get_type() << std::endl;
	this->obstacle_map[obstacle2->get_name()] = obstacle2;

	// blue cube
	Eigen::Vector3d obstacle_position3(17, 39, 5);
	auto obstacle3 = std::make_shared<Obstacle>("blue_cube", Obstacle_type::cube, true, 5.0, obstacle_position2);
	std::cout << "Add this static obstacle: " << obstacle3->get_position() << obstacle3->get_size() << obstacle3->get_name() << obstacle3->get_type() << std::endl;
	this->obstacle_map[obstacle3->get_name()] = obstacle3;
}

double MotionPlanner::distance_function(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b) {
	return hdi_plan_utils::get_distance(a->get_state(), b->get_state());
}

void MotionPlanner::setup() {
    //Eigen::Vector3d start_position(0,0,5);
    std::vector<double> start_position_param;
    this->pnh_.getParam("start_position", start_position_param);
	Eigen::Vector3d start_position(start_position_param[0],start_position_param[1],start_position_param[2]);
	std::cout << "The start position is set to: " << start_position(0) << start_position(1) << start_position(2) << std::endl;
    this->start_ = std::make_shared<RRTNode>(start_position, 0);
    //Eigen::Vector3d goal_position(20,20,5);
	std::vector<double> goal_position_param;
	this->pnh_.getParam("goal_position", goal_position_param);
	Eigen::Vector3d goal_position(goal_position_param[0],goal_position_param[1],goal_position_param[2]);
	std::cout << "The goal position is set to: " << goal_position(0) << goal_position(1) << goal_position(2) << std::endl;
    this->goal_ = std::make_shared<RRTNode>(goal_position, this->total_plan_time_);
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
    //this->pub_go_to_pose_.publish(msg);
	ROS_INFO("###Publish start position");
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
	bool choose_current_state_region = (rand() % 100) < 5;
	double x, y, z;

	if (choose_current_state_region) {
		double range = 1;
		x = this->quadrotor_state_(0) - range + (rand()/double(RAND_MAX)*2*range);
		y = this->quadrotor_state_(1) - range + (rand()/double(RAND_MAX)*2*range);
		z = this->quadrotor_state_(2) - range + (rand()/double(RAND_MAX)*2*range);
		//std::cout << "Select current state region." << std::endl;
	} else {
		double lower_bound = 0;
		//double upper_bound = 30;
        double upper_bound = 24;
		x = lower_bound + (rand()/double(RAND_MAX)*(upper_bound - lower_bound));

		//lower_bound = 2;
		//upper_bound = 4;
		y = lower_bound + (rand()/double(RAND_MAX)*(upper_bound - lower_bound));

        //lower_bound = 0;
        upper_bound = 3;
		z = lower_bound + (rand()/double(RAND_MAX)*(upper_bound - lower_bound));
	}

	Eigen::Vector3d random_position(x,y,z);
	auto random_node = std::make_shared<RRTNode>(random_position);
	//std::cout << "Generate random node position is: " << x << " " << y << " " << z << std::endl;
	return random_node;
}

double MotionPlanner::generate_random_time(const Eigen::Vector3d& state) {
	bool choose_time_by_distance = (rand() % 100) < 90;

	double min_time = this->total_plan_time_ - hdi_plan_utils::get_distance(state, this->goal_->get_state())/this->quadrotor_speed_;
	double max_time = this->total_plan_time_;

	double random_time;
	
	if (choose_time_by_distance) {
		random_time = min_time;
	} else {
		random_time = min_time + (rand()/double(RAND_MAX)*(max_time - min_time));
	}

	return random_time;
}

void MotionPlanner::quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    //ros::Duration(0.1).sleep();
    Eigen::Vector3d quadrotor_state(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    this->quadrotor_state_ = quadrotor_state;
    this->quadrotor_ = std::make_shared<RRTNode>(quadrotor_state);
    if (hdi_plan_utils::get_distance(quadrotor_state, this->goal_->get_state()) > 1) {
        this->solve();
    } else {
		std::cout << "The iteration number is: " << this->iteration_count << "The nodes number is: " << static_cast<double>(this->nearest_neighbors_tree_->size()) << std::endl;
        ROS_INFO("Already reach the goal. Will exit.");
		if (solve_time_path_file.is_open()) {
			solve_time_path_file.close();
		}
        this->sub_quadrotor_state_.shutdown();
    }
}

int MotionPlanner::get_human_id(const std::string human_name) {
	std::string delimiter = "_";
	std::string human_id_str = human_name.substr(human_name.find(delimiter)+1, human_name.find(delimiter) + delimiter.length()+1);
	int human_id = std::stoi(human_id_str);
	return human_id;
}

void MotionPlanner::obstacle_info_callback(const hdi_plan::obstacle_info::ConstPtr &msg) {
	Obstacle_type obstacle_type = static_cast<Obstacle_type>(msg->type);
	if (obstacle_type == hdi_plan::Obstacle_type::human) {
		int human_id = this->get_human_id(msg->name);
		if (std::find(this->human_id_list_.begin(), this->human_id_list_.end(),human_id)!=this->human_id_list_.end()) {
			return;
		}
		std::cout << "Initialize human_" << human_id << " now." << std::endl;
		this->human_id_list_.push_back(human_id);
		this->exist_human_ = true;
		ros::Time human_start_time = ros::Time::now();
		std::cout << "The human_" << human_id  << " start time is: " << static_cast<double>((human_start_time - this->start_time_).toSec()) << std::endl;
		Eigen::Vector2d human_start_position(msg->position.x, msg->position.y);
		this->human_map_tmp_[human_id] = std::make_shared<Human>(human_start_position, human_id, static_cast<double>((human_start_time - this->start_time_).toSec()));
		std::cout << "Human map tmp size is " << this->human_map_tmp_.size() << std::endl;
		std::cout << "Human map size is " << this->human_map_.size() << std::endl;
		return;
	}

	std::cout << "Motion Planner node: callback" << std::endl;
	std::string obstacle_name = msg->name;
	bool obstacle_operation = msg->operation;
	double obstacle_size = static_cast<double>(msg->size);
	Eigen::Vector3d obstacle_position(msg->position.x, msg->position.y, msg->position.z);
	auto obstacle = std::make_shared<Obstacle>(obstacle_name, obstacle_type, obstacle_operation, obstacle_size, obstacle_position);

	this->obstacle_update_info_list.push_back(obstacle);
}

void MotionPlanner::goal_point_callback(const geometry_msgs::Point::ConstPtr &msg) {
	Eigen::Vector3d goal_point(msg->x, msg->y, msg->z);
}

/*
void MotionPlanner::human_movement_callback_region_version(const geometry_msgs::Point::ConstPtr &msg) {
	Eigen::Vector2d human_position(msg->x, msg->y);
	this->human_position_ = human_position;
	this->exist_human_ = true;
	this->add_human_ = true;

	// human_vector is
	if (this->human_vector_.size() > 1) {
		this->human_vector_.pop();
	}
	this->human_vector_.push(human_position);
}*/

void MotionPlanner::human_movement_callback(const hdi_plan::obstacle_info::ConstPtr &msg) {
	int human_id = this->get_human_id(msg->name);
	if (!this->human_map_tmp_[human_id]->get_if_move()) {
		std::cout << "To get second pos of human_" << human_id << " ." << std::endl;
		Eigen::Vector2d human_position(msg->position.x, msg->position.y);
		this->human_map_tmp_[human_id]->set_second_position(human_position);
		this->human_map_tmp_[human_id]->set_if_move(true);
		this->human_map_[human_id] = this->human_map_tmp_[human_id];
		std::cout << "In movement: Human map tmp size is " << this->human_map_tmp_.size() << std::endl;
		std::cout << "In movement: Human map size is " << this->human_map_.size() << std::endl;
	}
}

bool MotionPlanner::solve() {
	ros::Duration(0.1).sleep();
	ros::Time solve_start_time = ros::Time::now();
    this->iteration_count += 1;
    //std::cout << "The iteration number is: " << this->iteration_count << "The nodes number is: " << static_cast<double>(this->nearest_neighbors_tree_->size()) << std::endl;
    /*
    if(!this->position_ready) {
        ROS_INFO("Set to start position");
        this->set_to_start_position();
		ros::Duration(2.0).sleep();
    }*/

    this->calculateRRG();

    this->update_obstacle();

    // sample random state
    // need to implement how to generate random state here, by adding the info of updating obstacles
    std::shared_ptr<RRTNode> random_node = generate_random_node();

    // find the closest node in the tree
    std::shared_ptr<RRTNode> nearest_node = this->nearest_neighbors_tree_->nearest(random_node);

    double distance = hdi_plan_utils::get_distance(random_node->get_state(), nearest_node->get_state());

    if (distance > this->max_distance_) {
        this->saturate(random_node, nearest_node, distance);
    }

    if (!this->extend(random_node)) {
        return false;
    }

    this->rewire_neighbors(random_node);
    this->reduce_inconsistency();

    if (this->iteration_count > 200 && (this->iteration_count % 100 == 1 || this->if_environment_change)) {
		if (this->update_solution_path()) {
			this->optimize_solution_path();
		}
    }

	double solve_time = (ros::Time::now() - solve_start_time).toSec();
	this->solve_time_path_file << solve_time << "\n";
	
    return true;
}

void MotionPlanner::update_obstacle() {
	//ROS_INFO("Update obstacle now");
	bool add_obstacle = false;
	bool remove_obstacle = false;

	for (auto it = this->obstacle_update_info_list.begin(); it != this->obstacle_update_info_list.end(); ++it) {
		std::shared_ptr<Obstacle> obstacle = *it;
		std::cout << "Obstacle info is: " << obstacle->get_position() << obstacle->get_size() << obstacle->get_name() << obstacle->get_type() << std::endl;
		if (!obstacle->get_operation()) {
			std::cout << "Remove this ob: " << obstacle->get_position() << obstacle->get_size() << obstacle->get_name() << obstacle->get_type() << std::endl;
			this->obstacle_map.erase(obstacle->get_name());
			this->remove_obstacle(obstacle);
			remove_obstacle = true;
		}
	}
	if (remove_obstacle) {
		this->if_environment_change = true;
		this->reduce_inconsistency();
	}

	for (auto it = this->obstacle_update_info_list.begin(); it != this->obstacle_update_info_list.end(); ++it) {
		std::shared_ptr<Obstacle> obstacle = *it;
		std::cout << "Obstacle info is: " << obstacle->get_position() << obstacle->get_size() << obstacle->get_name() << obstacle->get_type() << std::endl;
		if (obstacle->get_operation()) {
			std::cout << "Add this ob: " << obstacle->get_position() << obstacle->get_size() << obstacle->get_name() << obstacle->get_type() << std::endl;
			this->obstacle_map[obstacle->get_name()] = obstacle;
			this->add_obstacle(obstacle);
			add_obstacle = true;
		}
	}

	for (auto human : this->human_map_) {
		if ((!human.second->get_if_add()) && human.second->get_if_move()) {
			std::cout << "Adding human_" << human.first << " as an obstacle." << std::endl;
			this->add_human_as_obstacle(human.first);
			add_obstacle = true;
			human.second->set_if_add(true);
		}
	}

	if (add_obstacle) {
		this->if_environment_change = true;
		this->propogate_descendants();
		//this->verify_queue(v_bot);
		this->reduce_inconsistency();
	}

	this->obstacle_update_info_list.clear();
}

void MotionPlanner::remove_obstacle(const std::shared_ptr<Obstacle>& obstacle) {
	ROS_INFO("Start to removing the ob");
	std::vector<std::shared_ptr<RRTNode>> nodes_list;
	this->nearest_neighbors_tree_->list(nodes_list);

	// need to update this format to all for, and consider const reference
	for (auto node : nodes_list) {
		if (!check_if_node_inside_obstacle(obstacle, node) || check_if_node_inside_all_obstacles(node, true)) {
			continue;
		}
		this->update_lmc(node);
		if (node->get_lmc() != node->get_g_cost()) {
			this->verify_queue(node);
		}
	}
	/*
	// need to update this format to all for, and consider const reference
	for (auto node : nodes_list) {
		bool consider_human = true;
		if (obstacle->get_name() == "human") {
			consider_human = false;
		}
		if (!check_if_node_inside_obstacle(obstacle, node) || check_if_node_inside_all_obstacles(node, consider_human)) {
			continue;
		}
		this->update_lmc(node);
		if (node->get_lmc() != node->get_g_cost()) {
			this->verify_queue(node);
		}
	}*/
}

bool MotionPlanner::check_if_node_inside_obstacle(const std::shared_ptr<Obstacle>& obstacle, const std::shared_ptr<RRTNode>& node) {
	double distance = hdi_plan_utils::get_distance(obstacle->get_position(), node->get_state());
	if (distance - this->quadrotor_radius_ < obstacle->get_size()/2 + 1) {
        ROS_INFO("The node is inside obstacle");
		return true;
	}
	return false;
}

bool MotionPlanner::check_if_node_inside_all_obstacles(const std::shared_ptr<RRTNode>& node, bool consider_human) {
	if (consider_human && this->exist_human_) {
		for (auto human : this->human_map_) {
			if (human.second->check_if_node_inside_human(node)) {
				//ROS_INFO("#####The node is inside human, will ignore it.");
				return true;
			}
			
		}
	}

	bool result = std::any_of(this->obstacle_map.begin(), this->obstacle_map.end(), [this, node](auto obstacle){
		return this->check_if_node_inside_obstacle(obstacle.second, node);
	});

	return result;

	/*
	for (auto obstacle : this->obstacle_map) {
		if (this->check_if_node_inside_obstacle(obstacle.second, node)) return true;
	}
	return false;
	*/
}

/*
bool MotionPlanner::check_if_node_inside_human(const std::shared_ptr<RRTNode>& node) {
	if (!this->exist_human_) {
		return false;
	}

	Eigen::Vector3d state = node->get_state();
	double node_time = node->get_time();
	Eigen::Vector2d position = this->human_->predict_path(node_time);
	Eigen::Vector3d human_state(this->human_position_(0), this->human_position_(1), state(2));

 	return (state(2)<this->human_height_) && (this->compute_cost_with_weight(state, human_state)<this->human_block_distance_);
}*/

/*
double MotionPlanner::check_if_state_near_human(const Eigen::Vector3d& state) {
	if (!this->exist_human_) {
		return 1;
	}
	double weight = 5;
	return weight;
}

void MotionPlanner::add_human_as_obstacle(const Eigen::Vector2d& human_position) {
	std::vector<std::shared_ptr<RRTNode>> nodes_list;
	this->nearest_neighbors_tree_->list(nodes_list);
	for (auto it = nodes_list.begin(); it != nodes_list.end(); ++it) {
		std::shared_ptr<RRTNode> node = *it;
		if (check_if_node_inside_human(node)) {
			this->verify_orphan(node);
		} else if (check_if_state_near_human(node->get_state())>1 && !check_if_node_inside_all_obstacles(node, false)) {
			this->update_lmc(node);
			if (node->get_lmc() != node->get_g_cost()) {
				this->verify_queue(node);
			}
		}
	}
}*/

void MotionPlanner::add_human_as_obstacle(int human_id) {
	std::vector<std::shared_ptr<RRTNode>> nodes_list;
	this->nearest_neighbors_tree_->list(nodes_list);

	for (auto it = nodes_list.begin(); it != nodes_list.end(); ++it) {
		std::shared_ptr<RRTNode> node = *it;
		if (!this->human_map_[human_id]->check_if_node_inside_human(node)) {
			continue;
		}
		std::cout << "The colliding with human node position is: " << node->get_state() << std::endl;
		this->verify_orphan(node);
	}
}

void MotionPlanner::add_obstacle(const std::shared_ptr<Obstacle>& obstacle) {
	std::vector<std::shared_ptr<RRTNode>> nodes_list;
	this->nearest_neighbors_tree_->list(nodes_list);

	for (auto it = nodes_list.begin(); it != nodes_list.end(); ++it) {
		std::shared_ptr<RRTNode> node = *it;
		if (!check_if_node_inside_obstacle(obstacle, node)) {
			continue;
		}

		// publish the position of the blocked node to visualize
		geometry_msgs::Point node_msg;
		Eigen::Vector3d node_position = node->get_state();
		node_msg.x = node_position(0);
		node_msg.y = node_position(1);
		node_msg.z = node_position(2);
		this->pub_blocked_node_pos_.publish(node_msg);
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
	std::vector<std::shared_ptr<RRTNode>> orphan_node_append_list;
	double inf = std::numeric_limits<double>::infinity();
	for (auto it = this->orphan_node_list.begin(); it != this->orphan_node_list.end(); ++it) {
		std::shared_ptr<RRTNode> orphan_node = *it;
		orphan_node_append_list.insert(orphan_node_append_list.end(), orphan_node->children.begin(), orphan_node->children.end());
	}
	this->orphan_node_list.insert(this->orphan_node_list.end(), orphan_node_append_list.begin(), orphan_node_append_list.end());

	for (auto it = this->orphan_node_list.begin(); it != this->orphan_node_list.end(); ++it) {
		std::shared_ptr<RRTNode> orphan_node_2 = *it;

		std::vector<std::shared_ptr<RRTNode>> n_out;
		n_out.insert(n_out.end(), orphan_node_2->n0_out.begin(), orphan_node_2->n0_out.end());
		n_out.insert(n_out.end(), orphan_node_2->nr_out.begin(), orphan_node_2->nr_out.end());
		n_out.push_back(orphan_node_2->parent);

		for (auto it = n_out.begin(); it != n_out.end(); ++it) {
			std::shared_ptr<RRTNode> neighbor = *it;
			if (this->check_if_node_in_orphan_list(neighbor)) {
				continue;
			}
			neighbor->set_g_cost(inf, true);
			this->verify_queue(neighbor);
		}
	}

	for (auto it = this->orphan_node_list.begin(); it != this->orphan_node_list.end(); ++it) {
		std::shared_ptr<RRTNode> orphan_node_3 = *it;
		orphan_node_3->set_g_cost(inf, true);
		orphan_node_3->set_lmc(inf, true);
		if (orphan_node_3->parent != nullptr) {
			orphan_node_3->parent->children.erase(std::remove(orphan_node_3->parent->children.begin(), orphan_node_3->parent->children.end(), orphan_node_3), orphan_node_3->parent->children.end());
			orphan_node_3->parent = nullptr;
		}
	}

	this->orphan_node_list.clear();
}

bool MotionPlanner::check_if_node_in_orphan_list(const std::shared_ptr<RRTNode>& node) {
	for (auto it = this->orphan_node_list.begin(); it != this->orphan_node_list.end(); ++it) {
		if (*it == node) {
			return true;
		}
	}
	return false;
}

bool MotionPlanner::update_solution_path() {
    bool if_find_solution = false;

    std::shared_ptr<RRTNode> nearest_node_of_quadrotor = this->nearest_neighbors_tree_->nearest(this->quadrotor_);

    double cost_to_nearest_node = hdi_plan_utils::get_distance(nearest_node_of_quadrotor->get_state(), this->quadrotor_->get_state());

    //std::cout << "The distance from current position to nearest node: " << cost_to_nearest_node << std::endl;
    if (cost_to_nearest_node > this->max_distance_) {
        return if_find_solution; // return false, fail to find solution
    }
    //if (!this->edge_in_free_space_check(this->quadrotor_, nearest_node_of_quadrotor)) {
    //    return if_find_solution; // return false, fail to find solution
    //}

    this->solution_path.clear();
    std::shared_ptr<RRTNode> intermediate_node = nearest_node_of_quadrotor;

    while (intermediate_node == this->goal_ || intermediate_node->parent != nullptr) {
        this->solution_path.push_back(intermediate_node->get_state());
        if (intermediate_node == this->goal_) {
            if_find_solution = true;
            this->if_environment_change = false;
            break;
        }
        intermediate_node = intermediate_node->parent;
    }
	this->if_find_solution_ = if_find_solution;
    return if_find_solution;
}

void MotionPlanner::optimize_solution_path() {
	std::cout << "Now the solution path is found, the optimize part begins. The iteration number is: " << this->iteration_count << "The nodes number is: " << static_cast<double>(this->nearest_neighbors_tree_->size()) << std::endl;
	ros::Time chomp_start_time = ros::Time::now();
	ROS_INFO("Found the solution path, start to optimize");
	//std::cout << "The solution path contains: " << this->solution_path.size() << " points" << std::endl;
	//std::cout << "The obstacle number is " << this->obstacle_map.size() << std::endl;
	//std::cout << "The dynamic obstacle number is " << this->human_map_.size() << std::endl;
	auto chomp_trajectory = std::make_shared<ChompTrajectory>(this->solution_path, static_cast<double>((chomp_start_time - this->start_time_).toSec()), this->total_plan_time_, this->discretization_, this->quadrotor_speed_);
	auto chomp = std::make_shared<Chomp>(this->collision_threshold_, this->planning_time_limit_, this->max_iterations_,
									    this->max_iterations_after_collision_free_, this->learning_rate_, this->obstacle_cost_weight_, this->dynamic_obstacle_cost_weight_,
									    this->dynamic_collision_factor_, this->smoothness_cost_weight_, this->smoothness_cost_velocity_, this->smoothness_cost_acceleration_,
									    this->smoothness_cost_jerk_, this->ridge_factor_, this->min_clearence_, this->joint_update_limit_, this->quadrotor_radius_,
										chomp_trajectory, this->obstacle_map, this->human_map_);
	std::vector<Eigen::Vector3d> optimized_trajectory = chomp->get_optimized_trajectory();

	double chomp_process_time = (ros::Time::now() - chomp_start_time).toSec();
	std::cout << "The optimization time is: " << chomp_process_time << std::endl;

	hdi_plan::point_array trajectory_msg;
	int trajectory_size = optimized_trajectory.size();
	geometry_msgs::Point trajectory_point;
	std::string file_name = "data_" + std::to_string(this->path_file_num_) + ".txt";
	std::ofstream data_file (file_name);
	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d point = optimized_trajectory.at(i);
		trajectory_point.x = point(0);
		trajectory_point.y = point(1);
		trajectory_point.z = point(2);
		trajectory_msg.points.push_back(trajectory_point);
		data_file << point(0) << " " << point(1) << " " << point(2) << "\n";
		//ROS_INFO("publish trajectory is: x=%.2f, y=%.2f, z=%.2f", point(0), point(1), point(2));
	}
	if (data_file.is_open()) {
		data_file.close();
	}

	int ori_trajectory_size = this->solution_path.size();
	std::string ori_file_name = "ori_data_" + std::to_string(this->path_file_num_) + ".txt";
	std::ofstream ori_data_file (ori_file_name);
	for (int i = 0; i < ori_trajectory_size; i++) {
		Eigen::Vector3d ori_point = this->solution_path.at(i);
		ori_data_file << ori_point(0) << " " << ori_point(1) << " " << ori_point(2) << "\n";
		//ROS_INFO("Solution trajectory is: x=%.2f, y=%.2f, z=%.2f", ori_point(0), ori_point(1), ori_point(2));
	}
	if (ori_data_file.is_open()) {
		ori_data_file.close();
	}

	this->path_file_num_ += 1;

	ROS_INFO("Publish the trajectory from motion planner");

	std_msgs::Bool get_new_path_msg;
	get_new_path_msg.data = true;
	this->pub_get_new_path_.publish(get_new_path_msg);
	ros::Duration(1.0).sleep();
	this->pub_optimized_path_.publish(trajectory_msg);
}

void MotionPlanner::rewire_neighbors(std::shared_ptr<RRTNode> random_node) {
    //ROS_INFO("In rewire now");
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
        double new_lmc_if_choose_random_node_as_parent = random_node->get_lmc() + hdi_plan_utils::get_distance(neighbor->get_state(), random_node->get_state());
        if (neighbor->parent != this->goal_ && this->is_cost_better_than(new_lmc_if_choose_random_node_as_parent, neighbor->get_lmc())) {
            neighbor->set_lmc(new_lmc_if_choose_random_node_as_parent);
            this->make_parent_of(random_node, neighbor);
        }
        if (neighbor->get_g_cost() - neighbor->get_lmc() > this->epsilon_) {
            //ROS_INFO("In rewire: add to the priority queue");
            this->verify_queue(neighbor);
        }
    }
}

void MotionPlanner::make_parent_of(std::shared_ptr<RRTNode> parent_node, std::shared_ptr<RRTNode> node) {
	if (node->parent != nullptr) {
		node->parent->children.erase(std::remove(node->parent->children.begin(), node->parent->children.end(), node), node->parent->children.end());
	}
	node->parent = parent_node;
	parent_node->children.push_back(node);
}

void MotionPlanner::cull_neighbors(std::shared_ptr<RRTNode> random_node) {
    for (auto it = random_node->nr_out.begin(); it != random_node->nr_out.end(); ++it) {
        std::shared_ptr<RRTNode> neighbor = *it;
        if (this->rrg_r_ < hdi_plan_utils::get_distance(random_node->get_state(), neighbor->get_state()) && random_node->parent != neighbor) {
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
    //ROS_INFO("In reduce inconsistency");
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
    //ROS_INFO("In reduce inconsistency for env update");
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
        double inc_cost = hdi_plan_utils::get_distance(node->get_state(), neighbor->get_state());  // d(v,u)
        double cost = this->combine_cost(neighbor->get_lmc(), inc_cost); // d(v,u) + lmc(u)
        // if lmc(v) > d(v,u) + lmc(u)
        if (node->parent != this->goal_ && this->is_cost_better_than(cost, node->get_lmc())) {
            // change parent of v to u
			this->make_parent_of(neighbor, node);
            // update lmc(v)
            node->set_lmc(cost);
        }
    }
}

bool MotionPlanner::node_in_free_space_check(const std::shared_ptr<RRTNode>& random_node) {
	bool result = !this->check_if_node_inside_all_obstacles(random_node, true);
	//std::cout << "Node in free space check result is " << result << std::endl;
	return result;
}

bool MotionPlanner::edge_in_free_space_check(const std::shared_ptr<RRTNode>& node1, const std::shared_ptr<RRTNode>& node2) {
	bool result = (!this->check_if_node_inside_all_obstacles(node1, true)) && (!this->check_if_node_inside_all_obstacles(node2, true));
    //std::cout << "Edge in free space check result is " << result << std::endl;
    return result;
}

/*
bool MotionPlanner::check_if_edge_collide_human(const std::shared_ptr<RRTNode>& node1, const std::shared_ptr<RRTNode>& node2) {
	bool result = false;
	if (node1 && this->human_->check_if_node_inside_human(node1)) {
		result = true;
	} else if (node2 && this->human_->check_if_node_inside_human(node2)) {
		result = true;
	}

	if (result) ROS_INFO("########################The edge collides with human");
	//bool result = (this->human_->check_if_node_inside_human(node1)) || (this->human_->check_if_node_inside_human(node2));
	return result;
}*/

void MotionPlanner::calculateRRG() {
    auto num_of_nodes_in_tree = static_cast<double>(this->nearest_neighbors_tree_->size());
    this->rrg_r_ = std::min(this->max_distance_, this->r_rrt_ * std::pow(log(num_of_nodes_in_tree)/num_of_nodes_in_tree, 1/static_cast<double>(this->dimension_)));
    if (this->rrg_r_ == 0) {
        this->rrg_r_ = this->max_distance_ + 0.1;
    }
    //std::cout << "Tree size is: " << num_of_nodes_in_tree << std::endl;
    //std::cout << "The rrg_r_ is " << this->rrg_r_ << std::endl;
}

void MotionPlanner::saturate(std::shared_ptr<RRTNode> random_node, const std::shared_ptr<RRTNode>& nearest_node, double distance) const {
    // move the random node to distance = sigma to nearest node here, using interpolate
    Eigen::Vector3d random_node_state = random_node->get_state();
    Eigen::Vector3d nearest_node_state = nearest_node->get_state();
    double x = (random_node_state(0) - nearest_node_state(0)) * this->max_distance_ / distance + nearest_node_state(0);
    double y = (random_node_state(1) - nearest_node_state(1)) * this->max_distance_ / distance + nearest_node_state(1);
    double z = (random_node_state(2) - nearest_node_state(2)) * this->max_distance_ / distance + nearest_node_state(2);
    //ROS_INFO("Change the position of random node to be closer to nearest node");
    //std::cout << "Saturate, new position is: " << x << " " << y << " " << z << std::endl;
    random_node->set_state_by_value(x,y,z);
}

// inserting a new node
bool MotionPlanner::extend(std::shared_ptr<RRTNode> random_node) {
    //ROS_INFO("In extend now");
    if (!this->node_in_free_space_check(random_node)) {
        //ROS_INFO("Exit extend because 1st node space check");
        return false;
    }
    // v is random_node, u is neighbor
    // 1. find all nodes within shrinking hyperball in the nearest tree, and their distance
    this->update_neighbors_list(random_node);
    if (!this->find_best_parent(random_node)) {
        //ROS_INFO("### Cannot find best parent for this random node.");
        return false;
    }

    random_node->set_time(this->generate_random_time(random_node->get_state()));
    this->nearest_neighbors_tree_->add(random_node);
    random_node->parent->children.push_back(random_node);

    // publish the position of node to visualize
    geometry_msgs::Point node_msg;
    Eigen::Vector3d random_node_position = random_node->get_state();
    node_msg.x = random_node_position(0);
	node_msg.y = random_node_position(1);
	node_msg.z = random_node_position(2);
	this->pub_node_pos_.publish(node_msg);

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
    //std::cout << "Now is in find best parent." << std::endl;
    bool if_find_best_parent = false;
    for (auto it = random_node->nbh.begin(); it != random_node->nbh.end(); ++it) {
        std::shared_ptr<RRTNode> neighbor = it->first;
        // compute cost using this neighbor as a parent
        double inc_cost = hdi_plan_utils::get_distance(random_node->get_state(), neighbor->get_state());  // d(v,u)
        double cost = this->combine_cost(neighbor->get_lmc(), inc_cost); // d(v,u) + lmc(u)
        // if lmc(v) > d(v,u) + lmc(u)
        //std::cout << "Now check why always not find the best parent" << std::endl;
        //std::cout << "cost is " << cost << " . get_lmc is " << random_node->get_lmc() << " cost < lmc" << std::endl;
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
    this->nearest_neighbors_tree_->nearestR(random_node, this->rrg_r_ + 1, nbh);

    random_node->nbh.resize(nbh.size());
    // the default bool value of all added nodes are false, the bool value is the feasibility of edge as been tested
    std::transform(nbh.begin(), nbh.end(), random_node->nbh.begin(), [](std::shared_ptr<RRTNode> node) { return std::pair<std::shared_ptr<RRTNode>, bool>(node, false);});
}

double MotionPlanner::combine_cost(double cost1, double cost2) {
	return cost1 + cost2;
}

bool MotionPlanner::is_cost_better_than(double cost1, double cost2) {
	return cost1 < cost2;
}

/*
double MotionPlanner::compute_cost_human_cost_region_version(const Eigen::Vector3d& state1, const Eigen::Vector3d& state2) {
	double weight = 1;
	if (this->exist_human_) {
		double weight_1 = this->check_if_state_near_human(state1);
		double weight_2 = this->check_if_state_near_human(state2);
		weight = std::max(weight_1, weight_2);
	}
	return this->compute_cost_with_weight(state1, state2, weight);
}

double MotionPlanner::compute_cost_with_weight(const Eigen::Vector3d& state1, const Eigen::Vector3d& state2, double weight) {
	double cost = static_cast<double>(std::sqrt((state1 - state2).squaredNorm()));
	return cost * weight;
}*/

}
