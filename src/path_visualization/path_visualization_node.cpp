#include <ros/ros.h>

#include "path_visualization/path_visualization.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "start_quadrotor");
	hdi_plan::PathVisualization path_visualization(ros::NodeHandle(""), ros::NodeHandle("~"));
	ros::spin();
	return 0;
}