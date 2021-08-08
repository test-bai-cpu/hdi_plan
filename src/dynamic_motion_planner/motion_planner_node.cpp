#include <ros/ros.h>

#include "dynamic_motion_planner/motion_planner.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_planner");
    hdi_plan::MotionPlanner motion_planner(ros::NodeHandle(""), ros::NodeHandle("~"));
    ros::spin();
    //ros::AsyncSpinner spinner(0);
	//spinner.start();
	//ros::waitForShutdown();
    return 0;
}