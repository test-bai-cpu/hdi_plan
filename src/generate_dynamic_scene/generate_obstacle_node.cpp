#include <ros/ros.h>

#include "generate_dynamic_scene/generate_obstacle.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "generate_obstacle");
	hdi_plan::GenerateObstacle generate_obstacle(ros::NodeHandle(""), ros::NodeHandle("~"));
	ros::spin();
	return 0;
}