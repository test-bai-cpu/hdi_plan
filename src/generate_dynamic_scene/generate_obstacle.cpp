#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "utils/types.hpp"
#include <hdi_plan/obstacle_info.h>
#include "generate_dynamic_scene/obstacle.hpp"



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

int main(int argc, char **argv) {
	ros::init(argc, argv, "generate_obstacle");
	ros::NodeHandle nh;
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

	return 0;
}
