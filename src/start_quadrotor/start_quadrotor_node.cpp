#include <ros/ros.h>

#include "start_quadrotor/start_quadrotor.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "start_quadrotor");
    hdi_plan::StartQuadrotor start_quadrotor(ros::NodeHandle(""), ros::NodeHandle("~"));
    ros::spin();
    return 0;
}