#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "utils/types.hpp"
#include <hdi_plan/obstacle_info.h>
#include "generate_dynamic_scene/obstacle.hpp"


//test chomp
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

#include "dynamic_motion_planner/chomp_trajectory.hpp"
#include "dynamic_motion_planner/chomp.hpp"

#include "generate_dynamic_scene/human.hpp"
#include <hdi_plan/point_array.h>

/*
double get_distance(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2) {
	return static_cast<double>(std::sqrt((point1 - point2).squaredNorm()));
}

hdi_plan::obstacle_info get_obstacle_message(bool operation, double position_x = 100.0, double position_y = 100.0) {
	hdi_plan::obstacle_info obstacle_msg;
	obstacle_msg.name = "human";
	obstacle_msg.type = hdi_plan::Obstacle_type::human;
	obstacle_msg.operation = operation;
	obstacle_msg.size = 1;
	obstacle_msg.position.x = position_x;
	obstacle_msg.position.y = position_y;
	obstacle_msg.position.z = 0;

	return obstacle_msg;
}
*/
int main(int argc, char **argv) {
	ros::init(argc, argv, "generate_obstacle");
	ros::NodeHandle nh;

	//ros::Publisher pub_get_new_path_ = nh_.advertise<std_msgs::Bool>("hdi_plan/get_new_path", 1);
	//ros::Publisher pub_optimized_path_ = nh_.advertise<hdi_plan::point_array>("hdi_plan/full_trajectory", 1);

	/*
	ros::Publisher human_movement_pub = nh.advertise<geometry_msgs::Point>("hdi_plan/human_movement", 1);
	ros::Publisher update_human_obstacle_pub = nh.advertise<hdi_plan::obstacle_info>("hdi_plan/obstacle_info_topic", 1);

	// wait until human movement start
	ros::Duration(3.0).sleep();

	ros::Rate loop_rate(1);
	double period = static_cast<double>(loop_rate.expectedCycleTime().toSec());
	Eigen::Vector2d start_point(0, 10);
	Eigen::Vector2d goal_point(10, 10);
	Eigen::Vector2d current_point(start_point(0), start_point(1));
	double distance = get_distance(start_point, goal_point);
	double velocity = 1;

	int count = 0;

	while (get_distance(current_point, goal_point) > 0.5 * period * velocity) {
		if (count > 0) update_human_obstacle_pub.publish(get_obstacle_message(false));

		current_point(0) = (goal_point(0) - start_point(0)) * (velocity * count * period) / distance + start_point(0);
		current_point(1) = (goal_point(1) - start_point(1)) * (velocity * count * period) / distance + start_point(1);

		std::cout << "The human current position is, x: " << current_point(0) << " y: " << current_point(1) << std::endl;

		update_human_obstacle_pub.publish(get_obstacle_message(true,current_point(0),current_point(1)));
		geometry_msgs::Point msg;
		msg.x = current_point(0);
		msg.y = current_point(1);
		msg.z = 0;
		human_movement_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		count += 1;
	}
	*/

	std::vector<Eigen::Vector3d> solution_path;
	solution_path.resize(10);
	for (int i=0;i<10;i++) {
		Eigen::Vector3d s_point(1.0, i+1, 1.0);
		solution_path[i] = s_point;
	}

	
	std::map<std::string, std::shared_ptr<hdi_plan::Obstacle>> obstacle_map;
	std::string obstacle_name = "cube1";
	bool obstacle_operation = true;
	double obstacle_size = 1;
	Eigen::Vector3d obstacle_position(0, 5, 0);
	auto obstacle = std::make_shared<hdi_plan::Obstacle>(obstacle_name, hdi_plan::Obstacle_type::cube, obstacle_operation, obstacle_size, obstacle_position);
	obstacle_map[obstacle_name] = obstacle;

	ros::WallTime chomp_start_time = ros::WallTime::now();
	auto chomp_trajectory = std::make_shared<hdi_plan::ChompTrajectory>(solution_path);
	auto chomp = std::make_shared<hdi_plan::Chomp>(chomp_trajectory, obstacle_map);
	std::vector<Eigen::Vector3d> optimized_trajectory = chomp->get_optimized_trajectory();

	double chomp_process_time = (ros::WallTime::now() - chomp_start_time).toSec();
	std::cout << "The optimization time is: " << chomp_process_time << std::endl;

	int trajectory_size = optimized_trajectory.size();
	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d point = optimized_trajectory.at(i);
		ROS_INFO("the optimized trajectory is: x=%.2f, y=%.2f, z=%.2f", point(0), point(1), point(2));
	}

	/*
	hdi_plan::point_array trajectory_msg;
	int trajectory_size = optimized_trajectory.size();
	geometry_msgs::Point trajectory_point;
	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d point = optimized_trajectory.at(i);
		trajectory_point.x = point(0);
		trajectory_point.y = point(1);
		trajectory_point.z = point(2);
		trajectory_msg.points.push_back(trajectory_point);
		ROS_INFO("the optimized trajectory is: x=%.2f, y=%.2f, z=%.2f", point(0), point(1), point(2));
	}

	std_msgs::Bool get_new_path_msg;
	get_new_path_msg.data = true;
	pub_get_new_path_.publish(get_new_path_msg);
	
	pub_optimized_path_.publish(trajectory_msg);*/


	return 0;
}
