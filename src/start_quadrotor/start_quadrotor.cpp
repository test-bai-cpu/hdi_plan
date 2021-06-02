#include "start_quadrotor/start_quadrotor.hpp"

namespace hdi_plan {

StartQuadrotor::StartQuadrotor(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
:   nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::INDUSTRIAL),
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
    this->obstacle_info_sub_ = this->nh_.subscribe("hdi_plan/obstacle_info_topic", 1,
												   &StartQuadrotor::obstacle_callback, this);
    this->main_loop_timer_ = this->nh_.createTimer(ros::Rate(this->main_loop_freq_),
                                      &StartQuadrotor::main_loop_callback, this);
	this->arm_bridge_pub_ = nh_.advertise<std_msgs::Bool>("bridge/arm", 1);
	this->start_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);
	this->go_to_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1);
	this->max_velocity_pub_ = nh_.advertise<std_msgs::Float64>("autopilot/max_velocity", 1);


    spawn_quadrotor();

    // wait until the gazebo and unity are loaded
    ros::Duration(5.0).sleep();

    set_unity();
    connect_unity();

	start_quadrotor_bridge();

	ros::Duration(3.0).sleep();
}

StartQuadrotor::~StartQuadrotor() {}

void StartQuadrotor::spawn_quadrotor() {
    this->quad_ptr_ = std::make_shared<Quadrotor>();
    this->quad_state_.setZero();
    this->quad_ptr_->reset(this->quad_state_);
    Vector<3> quad_size(0.5, 0.5, 0.5);
    quad_ptr_->setSize(quad_size);
}

void StartQuadrotor::obstacle_callback(const hdi_plan::obstacle_info::ConstPtr &msg) {
    std::cout << "Now render the obstacle." << std::endl;
	std::string obstacle_id = msg->name;
    std::cout << "obstacle_id is " << obstacle_id << std::endl;
	Obstacle_type obstacle_type = static_cast<Obstacle_type>(msg->type);
    std::cout << "obstacle_type is " << obstacle_type << std::endl;
	bool obstacle_operation = msg->operation;
    std::cout << "obstacle_operation is " << obstacle_operation << std::endl;
	double obstacle_size = static_cast<double>(msg->size);
	std::cout << "obstacle_size is " << obstacle_size << std::endl;

	std::string prefab_id;
	switch (obstacle_type) {
		case Obstacle_type::cube: {
            std::cout << "test_cube" << std::endl;
			prefab_id = "test_cube";
			break;
		}
		case Obstacle_type::sphere: {
            std::cout << "Sphere" << std::endl;
			prefab_id = "Sphere";
			break;
		}
		case Obstacle_type::tank_1: {
			std::cout << "tank_1" << std::endl;
			prefab_id = "tank_1";
			break;
		}
		case Obstacle_type::human: {
			std::cout << "human" << std::endl;
			prefab_id = "human";
			break;
		}
		default: {
            std::cout << "Default: test_cube" << std::endl;
			prefab_id = "test_cube";
			break;
		}
	}

    std::shared_ptr<StaticObject> obstacle = std::make_shared<StaticObject>(obstacle_id, prefab_id);
	if (obstacle_operation) {
        std::cout << "Add the obstacle" << std::endl;
		obstacle->setPosition(Eigen::Vector3f((Scalar)msg->position.x, (Scalar)msg->position.y, (Scalar)msg->position.z));
	} else {
        std::cout << "Remove the obstacle" << std::endl;
		obstacle->setPosition(Eigen::Vector3f(100, 100, 100));
	}

	Vector<3> obstacle_size_local_scale(obstacle_size, obstacle_size, obstacle_size);
	obstacle->setSize(obstacle_size_local_scale);

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

void StartQuadrotor::start_quadrotor_bridge() {
	std_msgs::Bool arm_message;
	arm_message.data = true;
	this->arm_bridge_pub_.publish(arm_message);

	std_msgs::Empty start_message;
	this->start_pub_.publish(start_message);

	geometry_msgs::PoseStamped go_to_pose_msg;
	go_to_pose_msg.pose.position.x = 1.0;
	go_to_pose_msg.pose.position.y = 1.0;
	go_to_pose_msg.pose.position.z = 1.0;
	this->go_to_pose_pub_.publish(go_to_pose_msg);
}

}  // namespace hdi_plan