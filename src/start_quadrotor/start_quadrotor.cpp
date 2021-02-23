#include "start_quadrotor/start_quadrotor.hpp"

namespace hdi_plan {

StartQuadrotor::StartQuadrotor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
:   nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    main_loop_freq_(50.0) {

    // load parameters
    if (!load_params()) {
    ROS_WARN("[%s] Could not load all parameters.",
             this->pnh_.getNamespace().c_str());
    } else {
    ROS_INFO("[%s] Loaded all parameters.", this->pnh_.getNamespace().c_str());
    }

    this->quadrotor_state_sub_ = this->nh_.subscribe("hdi_plan/quadrotor_state", 1,
                                 &StartQuadrotor::pose_callback, this);
    this->obstacle_info_sub_ = this->nh_.subscribe("hdi_plan/obstacle_state", 1, &StartQuadrotor::obstacle_callback, this);
    this->main_loop_timer_ = this->nh_.createTimer(ros::Rate(this->main_loop_freq_),
                                      &StartQuadrotor::main_loop_callback, this);

    spawn_quadrotor();

    // wait until the gazebo and unity are loaded
    ros::Duration(5.0).sleep();

    set_unity();
    connect_unity();

}

StartQuadrotor::~StartQuadrotor() {}

void StartQuadrotor::spawn_quadrotor() {
    this->quad_ptr_ = std::make_shared<Quadrotor>();
    this->quad_state_.setZero();
    this->quad_ptr_->reset(this->quad_state_);
}

void StartQuadrotor::obstacle_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    std::string object_id = "obstacle";
    std::string prefab_id = "test_cube";
    std::shared_ptr<StaticGate> obstacle = std::make_shared<StaticGate>(object_id, prefab_id);
    obstacle->setPosition(Eigen::Vector3f((Scalar)msg->pose.pose.position.x, (Scalar)msg->pose.pose.position.y, (Scalar)msg->pose.pose.position.z));
    obstacle->setRotation(
            Quaternion((Scalar)msg->pose.pose.orientation.w, (Scalar)msg->pose.pose.orientation.x, (Scalar)msg->pose.pose.orientation.y, (Scalar)msg->pose.pose.orientation.z));
    this->unity_bridge_ptr_->addStaticObject(obstacle);

    if (this->unity_render_ && this->unity_ready_) {
        this->unity_bridge_ptr_->getRender(0);
        this->unity_bridge_ptr_->handleOutput();
    }
}

void StartQuadrotor::pose_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    this->quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
    this->quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
    this->quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
    this->quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
    this->quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
    this->quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
    this->quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

    this->quad_ptr_->setState(this->quad_state_);

    if (this->unity_render_ && this->unity_ready_) {
      this->unity_bridge_ptr_->getRender(0);
      this->unity_bridge_ptr_->handleOutput();
    }
}

void StartQuadrotor::set_unity() {
    if (!this->unity_render_ && this->unity_bridge_ptr_ == nullptr) {
        this->unity_bridge_ptr_ = UnityBridge::getInstance();
        this->unity_bridge_ptr_->addQuadrotor(this->quad_ptr_);
        this->unity_render_ = true;
    ROS_INFO("[%s] Unity Bridge is created.", this->pnh_.getNamespace().c_str());
    }
}

void StartQuadrotor::connect_unity() {
    if (this->unity_render_ && this->unity_bridge_ptr_) {
        this->unity_ready_ = this->unity_bridge_ptr_->connectUnity(this->scene_id_);
    }
}

void StartQuadrotor::main_loop_callback(const ros::TimerEvent &event) {
  // empty
}

bool StartQuadrotor::load_params() {
    // load parameters
    quadrotor_common::getParam("main_loop_freq", this->main_loop_freq_, this->pnh_);
    return true;
}


}  // namespace hdi_plan