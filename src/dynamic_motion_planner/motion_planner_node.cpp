#include <ros/ros.h>

#include "dynamic_motion_planner/motion_planner.hpp"

// ompl
#include <ompl/base/spaces/SE3StateSpace.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_planner");
    hdi_plan::MotionPlanner motion_planner(ros::NodeHandle(""), ros::NodeHandle("~"));
    //ompl::base::RealVectorBounds bounds = motion_planner.get_space_info();
    int dim = motion_planner.get_dim();
    std::cout << "#########################" << dim << std::endl;
    ros::spin();
    return 0;
}