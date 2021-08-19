#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>
#include <iostream>
#include "utils/types.hpp"
#include "generate_dynamic_scene/obstacle.hpp"
#include "generate_dynamic_scene/human.hpp"
#include <hdi_plan/obstacle_info.h>
#include <hdi_plan/point_array.h>

#include <stack>
#include <queue>
#include <vector>
#include <memory>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <time.h>
#include <string>
#include <map>
#include <queue>
#include <algorithm>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "dynamic_motion_planner/chomp_trajectory.hpp"
#include "dynamic_motion_planner/chomp.hpp"
#include "generate_dynamic_scene/human.hpp"
#include "utils/utility_functions.hpp"
#include <hdi_plan/point_array.h>

#include <iostream>
#include <fstream>

namespace hdi_plan {

class GenerateObstacle {
public:
	GenerateObstacle(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
	~GenerateObstacle();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Publisher human_movement_pub_;
	ros::Publisher update_human_obstacle_pub_;
	ros::Publisher pub_optimized_path_;
	ros::Publisher pub_get_new_path_;

    void publish_human_movement_1();
	void publish_human_movement_2();
	void publish_obstacle();
	void publish_obstacle2();
	void remove_obstacle();
    double get_distance(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2);
    hdi_plan::obstacle_info get_obstacle_message(bool operation, int human_id, double position_x = 100.0, double position_y = 100.0);
    /*
	void publish_trajectory();
	bool load_params();


	// chomp trajectory
	double discretization_;

	// chomp
	double quadrotor_radius_;
	double quadrotor_speed_;
	double total_plan_time_;
	double collision_threshold_;
	double planning_time_limit_;
	int max_iterations_;
	int max_iterations_after_collision_free_;
	double learning_rate_;
	double obstacle_cost_weight_;
	double dynamic_obstacle_cost_weight_;
	double dynamic_collision_factor_;
	double smoothness_cost_weight_;
	double smoothness_cost_velocity_;
	double smoothness_cost_acceleration_;
	double smoothness_cost_jerk_;
	double ridge_factor_;
	double min_clearence_;
	double joint_update_limit_;*/
};
}