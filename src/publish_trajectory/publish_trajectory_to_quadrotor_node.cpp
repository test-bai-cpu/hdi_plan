#include <ros/ros.h>

#include "publish_trajectory/publish_trajectory_to_quadrotor.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "publish_trajectory_to_quadrotor");
	hdi_plan::PublishTrajectory publish_trajectory(ros::NodeHandle(""), ros::NodeHandle("~"));
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();
	return 0;
}