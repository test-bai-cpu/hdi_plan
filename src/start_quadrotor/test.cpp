
// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_obstacle.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// trajectory
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>

//utils
#include <Eigen/Dense>

using namespace flightlib;

void modify_quad_state(QuadState& quad_state, const Eigen::Vector3d& point) {
    quad_state.x[QS::POSX] = (Scalar)point(0);
    quad_state.x[QS::POSY] = (Scalar)point(1);
    quad_state.x[QS::POSZ] = (Scalar)point(2);
}

quadrotor_common::TrajectoryPoint generate_start_state() {
    quadrotor_common::TrajectoryPoint desired_pose;
    return desired_pose;
}

int main(int argc, char *argv[]) {
    // initialize ROS
    ros::init(argc, argv, "quadrotor_start");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    // Flightmare(Unity3D)
    std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();

    // Initialize unity quadrotor
    std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>();
    QuadState quad_state;
    quad_state.setZero();
    quad_ptr->reset(quad_state);
    Vector<3> quad_size(0.5, 0.5, 0.5);
    quad_ptr->setSize(quad_size);

    // Spawn quadrotor
    unity_bridge_ptr->addQuadrotor(quad_ptr);

    // connect unity
    bool unity_ready = unity_bridge_ptr->connectUnity(UnityScene::INDUSTRIAL);


    // Execute the trajectory
    FrameID frame_id = 0;
    //ros::Time t0 = ros::Time::now();

    while (ros::ok() && unity_ready) {
        Eigen::Vector3d start_point(0, 0, 0);
        Eigen::Vector3d point1(0, 10, 2.5);
        Eigen::Vector3d point2(5, 0, 2.5);
        Eigen::Vector3d point3(0, -10, 2.5);
        Eigen::Vector3d point4(-5, 0, 2.5);

        if (frame_id == 0) {
            modify_quad_state(quad_state, start_point);
        }else if (frame_id == 10) {
            modify_quad_state(quad_state, point1);
        }else if (frame_id == 20) {
            modify_quad_state(quad_state, point2);
        }else if (frame_id == 30) {
            modify_quad_state(quad_state, point3);
        }else if (frame_id == 40) {
            modify_quad_state(quad_state, point4);
        }

        quad_ptr->setState(quad_state);
        unity_bridge_ptr->getRender(frame_id);
        unity_bridge_ptr->handleOutput();

        frame_id += 1;
    }

    return 0;
}
