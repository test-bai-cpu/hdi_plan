#include "start_quadrotor/start_quadrotor.hpp"

namespace hdi_plan {

StartQuadrotor::StartQuadrotor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
:   nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {

    state_estimate_sub_ = nh_.subscribe("hdi_plan/state_estimate", 1,
                                 &StartQuadrotor::pose_callback, this);
    obstacle_info_sub_ = nh_.subscribe("hdi_plan/test1", 1, &StartQuadrotor::obstacle_callback, this);
    main_loop_timer_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                      &StartQuadrotor::main_loop_callback, this);

    spawn_quadrotor();

    // wait until the gazebo and unity are loaded
    ros::Duration(5.0).sleep();

    set_unity();
    connect_unity();

}

StartQuadrotor::~StartQuadrotor() {}

void StartQuadrotor::obstacle_update_callback() {


}

void StartQuadrotor::spawn_quadrotor() {
    // quad initialization
    quad_ptr_ = std::make_shared<Quadrotor>();

    Vector<3> B_r_BC(0.0, 0.0, 0.3);
    Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
    std::cout << R_BC << std::endl;

    // initialization
    quad_state_.setZero();
    quad_ptr_->reset(quad_state_);
}

void StartQuadrotor::obstacle_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    std::string object_id = "obstacle";
    std::string prefab_id = "test_cube";
    std::shared_ptr<StaticGate> obstalce = std::make_shared<StaticGate>(object_id, prefab_id);
    obstalce->setPosition(Eigen::Vector3f((Scalar)msg->pose.pose.position.x, (Scalar)msg->pose.pose.position.y, (Scalar)msg->pose.pose.position.z));
    obstalce->setRotation(
            Quaternion((Scalar)msg->pose.pose.orientation.w, (Scalar)msg->pose.pose.orientation.x, (Scalar)msg->pose.pose.orientation.y, (Scalar)msg->pose.pose.orientation.z));
    unity_bridge_ptr_->addStaticObject(obstalce);

    if (unity_render_ && unity_ready_) {
        unity_bridge_ptr_->getRender(0);
        unity_bridge_ptr_->handleOutput();
    }
}

void StartQuadrotor::pose_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();
  }
}

void StartQuadrotor::set_unity() {
  if (!unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    unity_render_ = true;
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
}

void StartQuadrotor::connect_unity() {
  if (unity_render_ && unity_bridge_ptr_) {
      unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  }
}

bool StartQuadrotor::load_params() {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace hdi_plan