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

    // load parameters
    /*
    if (!load_params()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
    } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
    }*/
    state_estimate_sub_ = nh_.subscribe("hdi_plan/state_estimate", 1,
                                 &StartQuadrotor::pose_callback, this);
    test_sub_ = nh_.subscribe("/hdi_plan/test1", 1, &StartQuadrotor::test1_callback, this);
    main_loop_timer_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                      &StartQuadrotor::main_loop_callback, this);

    spawn_quadrotor();

    // connect unity
    set_unity();
    connect_unity();

    // obstacle info subscriber call backs
    //obstacle_info_sub_ = nh_.subscribe("hdi_plan/obstalce_info", 1,
    //                             &StartQuadrotor::obstacle_update_callback, this);


    // wait until the gazebo and unity are loaded
    // ros::Duration(5.0).sleep();
    // keep_rendering();
}

StartQuadrotor::~StartQuadrotor() {}

void StartQuadrotor::obstacle_update_callback() {


}

void StartQuadrotor::get_unity_connect_info() {
  std::cout << "unity_ready_ is" << unity_ready_ << std::endl;
  std::cout << "unity_render_ is" << unity_render_ << std::endl;
}

void StartQuadrotor::render_new_frame() {
  std::string object_id = "check_ob1";
  std::string prefab_id = "rpg_gate";
  std::shared_ptr<StaticGate> obstalce1 = std::make_shared<StaticGate>(object_id, prefab_id);
  obstalce1->setPosition(Eigen::Vector3f(5, 0, 2.5));
  obstalce1->setRotation(
    Quaternion(std::cos(0.5 * M_PI_2), 0.0, 0.0, std::sin(0.5 * M_PI_2)));
  unity_bridge_ptr_->addStaticObject(obstalce1);
  std::cout << "add obstacle, before rendering, unity_ready_ is" << unity_ready_ << std::endl;
  std::cout << "add obstacle, before rendering, unity_render_ is" << unity_render_ << std::endl;
  FrameID frame_id = 0;
  while (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(frame_id);
    unity_bridge_ptr_->handleOutput();
    frame_id += 1;
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

void StartQuadrotor::test1_callback(const std_msgs::String& msg) {
  ROS_INFO("I heard: [%s]", msg.data.c_str());
}


void StartQuadrotor::main_loop_callback(const ros::TimerEvent &event) {
  // empty
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

void StartQuadrotor::spawn_quadrotor() {
    // quad initialization
    quad_ptr_ = std::make_shared<Quadrotor>();

    // add mono camera
    rgb_camera_ = std::make_shared<RGBCamera>();
    Vector<3> B_r_BC(0.0, 0.0, 0.3);
    Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
    std::cout << R_BC << std::endl;
    rgb_camera_->setFOV(90);
    rgb_camera_->setWidth(720);
    rgb_camera_->setHeight(480);
    rgb_camera_->setRelPose(B_r_BC, R_BC);
    quad_ptr_->addRGBCamera(rgb_camera_);

    // initialization
    quad_state_.setZero();
    quad_ptr_->reset(quad_state_);
}
void StartQuadrotor::keep_rendering() {
  std::cout << "test1" << std::endl;
}
}  // namespace hdi_plan