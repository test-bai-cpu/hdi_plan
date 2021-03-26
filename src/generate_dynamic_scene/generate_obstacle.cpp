#include <ros/ros.h>

#include "generate_dynamic_scene/obstacle.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "generate_obstacle");
	ros::spin();
	return 0;
}