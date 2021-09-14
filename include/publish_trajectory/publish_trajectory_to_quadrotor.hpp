#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

//autopilot
#include <quadrotor_msgs/Trajectory.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_common/trajectory_point.h>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

#include "utils/utility_functions.hpp"
#include <hdi_plan/point_array.h>
#include <hdi_plan/obstacle_info.h>


namespace hdi_plan {

class PublishTrajectory {
public:
	PublishTrajectory(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
	~PublishTrajectory();

private:
	// ros nodes
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	// subscriber
	ros::Subscriber quadrotor_state_sub_;
	ros::Subscriber trajectory_sub_;
	ros::Subscriber get_new_path_sub_;

	// publisher
	ros::Publisher arm_bridge_pub_;
	ros::Publisher start_pub_;
	ros::Publisher hover_pub_;
	ros::Publisher go_to_pose_pub_;
	ros::Publisher max_velocity_pub_;
	ros::Publisher pub_solution_path_;
	ros::Publisher trajectory_pub_;

	Eigen::Vector3d quadrotor_state_;
	Eigen::Vector3d goal_state_;
	bool if_get_new_path_ = false;

	void start_quadrotor_bridge();
	void quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg);
	void get_new_path_callback(const std_msgs::Bool::ConstPtr &msg);
	void trajectory_callback(const hdi_plan::point_array::ConstPtr &msg);

	std::ofstream executed_path_file;
};

}  // namespace flightros 

/*
	ros::Publisher trajectory_pub_;
	ros::Publisher pub_get_new_path_;
	ros::Subscriber trigger_sub_;
	void publish_trajectory();
	void publish_another_trajectory();
	void trigger_callback(const std_msgs::Empty::ConstPtr &msg);*/