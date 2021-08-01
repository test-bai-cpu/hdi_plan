#include "publish_trajectory/publish_trajectory_to_quadrotor.hpp"

namespace hdi_plan {

PublishTrajectory::PublishTrajectory(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
	:nh_(nh),
	 pnh_(pnh){

	arm_bridge_pub_ = nh_.advertise<std_msgs::Bool>("bridge/arm", 1);
	start_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);

	go_to_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 100);
	max_velocity_pub_ = nh_.advertise<std_msgs::Float64>("autopilot/max_velocity", 1);
	trajectory_sub_ = nh_.subscribe("hdi_plan/full_trajectory", 1, &PublishTrajectory::trajectory_callback, this);
	//pub_solution_path_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
	pub_solution_path_ = nh_.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1);
	get_new_path_sub_ = nh_.subscribe("hdi_plan/get_new_path", 1, &PublishTrajectory::get_new_path_callback, this);

	quadrotor_state_sub_ = nh_.subscribe("hdi_plan/quadrotor_state", 1, &PublishTrajectory::quadrotor_state_callback, this);

	//Eigen::Vector3d goal_state(20,1,2);
	std::vector<double> goal_position_param;
	this->pnh_.getParam("goal_position", goal_position_param);
	Eigen::Vector3d goal_state(goal_position_param[0],goal_position_param[1],goal_position_param[2]);
	this->goal_state_ = goal_state;

	ros::Duration(5.0).sleep();
	start_quadrotor_bridge();

	std::string file_name = "executed_path.txt";
	executed_path_file.open(file_name);
}

PublishTrajectory::~PublishTrajectory() {}

void PublishTrajectory::quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg) {
	quadrotor_state_(0) = msg->pose.pose.position.x;
	quadrotor_state_(1) = msg->pose.pose.position.y;
	quadrotor_state_(2) = msg->pose.pose.position.z;
}

void PublishTrajectory::start_quadrotor_bridge() {
	ROS_INFO("######  Start the quadrotor bridge");
	std_msgs::Bool arm_message;
	arm_message.data = true;
	this->arm_bridge_pub_.publish(arm_message);

	std_msgs::Empty empty_message;
	this->start_pub_.publish(empty_message);
	ros::Duration(7.0).sleep();

	std::vector<double> start_position_param;
	this->pnh_.getParam("start_position", start_position_param);
	geometry_msgs::PoseStamped go_to_pose_msg;
	go_to_pose_msg.pose.position.x = start_position_param[0];
	go_to_pose_msg.pose.position.y = start_position_param[1];
	go_to_pose_msg.pose.position.z = start_position_param[2];
	
	this->pub_solution_path_.publish(go_to_pose_msg);
	//this->go_to_pose_pub_.publish(go_to_pose_msg);
}

void PublishTrajectory::get_new_path_callback(const std_msgs::Bool::ConstPtr &msg) {
	this->if_get_new_path_ = msg->data;
}

void PublishTrajectory::trajectory_callback(const hdi_plan::point_array::ConstPtr &msg) {
	ROS_INFO("In the trajectory callback");
	int trajectory_size = msg->points.size();
	this->if_get_new_path_ = false;

	geometry_msgs::PoseStamped go_to_pose_msg;
	bool find_current_position = false;
	for (int i = 0; i < trajectory_size; i++) {
		if ((i > 4) && (i%5 != 1) && (i != trajectory_size-1)) continue;
		
		if (this->if_get_new_path_) {
			ROS_INFO("Now break the previous callback");
			break;
		}

		if (hdi_plan_utils::get_distance(quadrotor_state_, this->goal_state_) < 0.5) {
			ROS_INFO("Already reach the goal area, in publish trajectory node. Stop publishing the trajectory.");
			i = trajectory_size-1;
		}
		
		if ((!find_current_position) && (i<20)) {
			Eigen::Vector3d traj_point(msg->points[i].x, msg->points[i].y, msg->points[i].z);
			double distance = hdi_plan_utils::get_distance(traj_point, this->quadrotor_state_);
			if (distance < 0.5) find_current_position = true;
			continue;
		}

		go_to_pose_msg.pose.position.x = msg->points[i].x;
		go_to_pose_msg.pose.position.y = msg->points[i].y;
		go_to_pose_msg.pose.position.z = msg->points[i].z;
		//this->go_to_pose_pub_.publish(go_to_pose_msg);
		this->pub_solution_path_.publish(go_to_pose_msg);
		//std::cout << "The executed position is: " << msg->points[i].x << " " << msg->points[i].y << " " << msg->points[i].z << std::endl;
		this->executed_path_file << msg->points[i].x << " " << msg->points[i].y << " " << msg->points[i].z << "\n";
		ros::Duration(0.5).sleep();
	}
}

}

/*
	trajectory_pub_ = nh_.advertise<hdi_plan::point_array>("hdi_plan/full_trajectory", 1);
trigger_sub_ = nh_.subscribe("hdi_plan/trigger", 1, &PublishTrajectory::trigger_callback, this);
	pub_get_new_path_ = nh_.advertise<std_msgs::Bool>("hdi_plan/get_new_path", 1);
void PublishTrajectory::trigger_callback(const std_msgs::Empty::ConstPtr &msg) {
	this->publish_trajectory();
}
void PublishTrajectory::publish_another_trajectory() {
	ROS_INFO("check into another publish trajectory");
	quadrotor_common::Trajectory trajectory;
	for (int i=0; i<10; i++) {
		quadrotor_common::TrajectoryPoint point = quadrotor_common::TrajectoryPoint();
		Eigen::Vector3d position(1,i+1,10);
		point.position = position;
		trajectory.points.push_back(point);
	}

	quadrotor_msgs::Trajectory msg = trajectory.toRosMessage();
	this->trajectory_pub_.publish(msg);
}

void PublishTrajectory::publish_trajectory() {
	ROS_INFO("check into publish trajectory");
	std::vector<Eigen::Vector3d> optimized_trajectory;
	for (int i = 0; i < 10; i++) {
		Eigen::Vector3d point1(i, i+1, i+2);
		optimized_trajectory.push_back(point1);
	}

	hdi_plan::point_array trajectory_msg;
	int trajectory_size = optimized_trajectory.size();
	geometry_msgs::Point trajectory_point;
	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d point = optimized_trajectory.at(i);
		trajectory_point.x = point(0);
		trajectory_point.y = point(1);
		trajectory_point.z = point(2);
		trajectory_msg.points.push_back(trajectory_point);
	}

	std_msgs::Bool get_new_path_msg;
	get_new_path_msg.data = true;
	this->pub_get_new_path_.publish(get_new_path_msg);

	this->trajectory_pub_.publish(trajectory_msg);
	ROS_INFO("Finish publish trajectory");
}
*/