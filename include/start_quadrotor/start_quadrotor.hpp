#pragma once

#include <memory>

// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "std_msgs/String.h"

// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/sensors/rgb_camera.hpp"
#include "flightlib/bridges/unity_message_types.hpp"

using namespace flightlib;

namespace hdi_plan {

class StartQuadrotor {
public:
    StartQuadrotor(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~StartQuadrotor();

    //debug functions
    void get_unity_connect_info();
    void render_new_frame();

    void set_unity();
    void connect_unity();

private:
    // ros nodes
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // subscriber
    ros::Subscriber state_estimate_sub_;
    ros::Subscriber test_sub_;

    // main loop timer
    ros::Timer main_loop_timer_;

    // unity quadrotor
    std::shared_ptr<Quadrotor> quad_ptr_;
    std::shared_ptr<RGBCamera> rgb_camera_;
    QuadState quad_state_;

    // Flightmare(Unity3D)
    SceneID scene_id_;
    bool unity_ready_;
    bool unity_render_;
    uint16_t receive_id_;
    std::shared_ptr<UnityBridge> unity_bridge_ptr_;
    RenderMessage_t unity_output_;
    

    // auxiliary variables
    Scalar main_loop_freq_;

    bool load_params();

    void spawn_quadrotor();
    void keep_rendering();

    // callbacks
    void main_loop_callback(const ros::TimerEvent& event);
    void pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void obstacle_update_callback();
    void test1_callback(const std_msgs::String& msg);
};

}  // namespace flightros
