#include "generate_dynamic_scene/generate_obstacle.hpp"

namespace hdi_plan {

GenerateObstacle::GenerateObstacle(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
		: nh_(nh),
		  pnh_(pnh) {
	this->human_movement_pub_ = nh_.advertise<hdi_plan::obstacle_info>("hdi_plan/human_movement", 1);
	this->update_human_obstacle_pub_ = nh_.advertise<hdi_plan::obstacle_info>("hdi_plan/obstacle_info_topic", 1);

	// wait until human movement start
	//ros::Duration(24.0 + 40.0).sleep();
	//this->publish_obstacle();
	//ros::Duration(5.0).sleep();
	//this->remove_obstacle();
	ros::Duration(24.0 + 40.0).sleep();
	this->publish_human_movement_1();
}

GenerateObstacle::~GenerateObstacle() = default;

hdi_plan::obstacle_info GenerateObstacle::get_obstacle_message(bool operation, int human_id, double position_x, double position_y) {
	hdi_plan::obstacle_info obstacle_msg;
	obstacle_msg.name = "human_" + std::to_string(human_id);
	obstacle_msg.type = hdi_plan::Obstacle_type::human;
	obstacle_msg.operation = operation;
	obstacle_msg.size = 1;
	obstacle_msg.position.x = position_x;
	obstacle_msg.position.y = position_y;
	obstacle_msg.position.z = 0;

	return obstacle_msg;
}

void GenerateObstacle::publish_obstacle() {
	hdi_plan::obstacle_info obstacle_msg;
	obstacle_msg.name = "sphere1";
	obstacle_msg.type = hdi_plan::Obstacle_type::sphere;
	obstacle_msg.operation = true;
	obstacle_msg.size = 3;
	obstacle_msg.position.x = 10.0;
	obstacle_msg.position.y = 10.0;
	obstacle_msg.position.z = 2.0;
	update_human_obstacle_pub_.publish(obstacle_msg);
}

void GenerateObstacle::remove_obstacle() {
	hdi_plan::obstacle_info obstacle_msg;
	obstacle_msg.name = "sphere1";
	obstacle_msg.type = hdi_plan::Obstacle_type::sphere;
	obstacle_msg.operation = false;
	obstacle_msg.size = 3;
	obstacle_msg.position.x = 10.0;
	obstacle_msg.position.y = 10.0;
	obstacle_msg.position.z = 2.0;
	update_human_obstacle_pub_.publish(obstacle_msg);
}

void GenerateObstacle::publish_obstacle2() {
	hdi_plan::obstacle_info obstacle_msg;
	obstacle_msg.name = "sphere2";
	obstacle_msg.type = hdi_plan::Obstacle_type::sphere;
	obstacle_msg.operation = true;
	obstacle_msg.size = 5;
	obstacle_msg.position.x = 10.0;
	obstacle_msg.position.y = 5.0;
	obstacle_msg.position.z = 2.0;
	update_human_obstacle_pub_.publish(obstacle_msg);
}

void GenerateObstacle::publish_human_movement_1() {
	Eigen::Vector2d start_point(10.0, 23.0);
	Eigen::Vector2d goal_point(10.0, 0.0);
	Eigen::Vector2d current_point(start_point(0), start_point(1));
	double distance = hdi_plan_utils::get_distance_2d(start_point, goal_point);
	double velocity = 1;
	double period = 1;
	int human_id = 1;
	int count = 0;
	ROS_INFO("###Generate Obstacle: Start to spawn a moving obstacle 1.");
	while (hdi_plan_utils::get_distance_2d(current_point, goal_point) > 0.5 * period * velocity) {
		if (count > 0) update_human_obstacle_pub_.publish(get_obstacle_message(false, human_id));

		current_point(0) = (goal_point(0) - start_point(0)) * (velocity * count * period) / distance + start_point(0);
		current_point(1) = (goal_point(1) - start_point(1)) * (velocity * count * period) / distance + start_point(1);

		std::cout << "Sending pos: " << current_point(0) << " " << current_point(1) << std::endl;
		update_human_obstacle_pub_.publish(get_obstacle_message(true, human_id, current_point(0), current_point(1)));
		if (count > 0) human_movement_pub_.publish(get_obstacle_message(true, human_id, current_point(0), current_point(1)));

		ros::Duration(period).sleep();
		count += 1;
	}
	//ROS_INFO("###Finish Generate Obstacle: Start to spawn a moving obstacle 1.");
}

void GenerateObstacle::publish_human_movement_2() {
	Eigen::Vector2d start_point(15.0, 1.0);
	Eigen::Vector2d goal_point(15.0, 30.0);
	Eigen::Vector2d current_point(start_point(0), start_point(1));
	double distance = hdi_plan_utils::get_distance_2d(start_point, goal_point);
	double velocity = 1;
	double period = 1;
	int human_id = 2;
	int count = 0;
	ROS_INFO("###Generate Obstacle: Start to spawn a moving obstacle 2.");
	while (hdi_plan_utils::get_distance_2d(current_point, goal_point) > 0.5 * period * velocity) {
		if (count > 0) update_human_obstacle_pub_.publish(get_obstacle_message(false, human_id));

		current_point(0) = (goal_point(0) - start_point(0)) * (velocity * count * period) / distance + start_point(0);
		current_point(1) = (goal_point(1) - start_point(1)) * (velocity * count * period) / distance + start_point(1);

		update_human_obstacle_pub_.publish(get_obstacle_message(true, human_id, current_point(0), current_point(1)));
		if (count > 0) human_movement_pub_.publish(get_obstacle_message(true, human_id, current_point(0), current_point(1)));

		ros::Duration(period).sleep();
		count += 1;
	}
}

}
/*

int main(int argc, char **argv) {
	ros::init(argc, argv, "generate_obstacle");
	ros::NodeHandle nh;

	//ros::Publisher pub_get_new_path_ = nh.advertise<std_msgs::Bool>("hdi_plan/get_new_path", 1);
	ros::Publisher pub_optimized_path_ = nh.advertise<hdi_plan::point_array>("hdi_plan/full_trajectory", 1);
	ros::Rate loop_rate(10);
	ROS_INFO("test1");
	std::vector<Eigen::Vector3d> solution_path;
	solution_path.resize(5);
	Eigen::Vector3d point_1(0.0, 1.0, 1.0);
	solution_path[0] = point_1;
	Eigen::Vector3d point_2(1.0, 2.0, 1.0);
	solution_path[1] = point_2;
	Eigen::Vector3d point_3(2.0, 0.0, 1.0);
	solution_path[2] = point_3;
	Eigen::Vector3d point_4(3.0, 2.0, 1.0);
	solution_path[3] = point_4;
	Eigen::Vector3d point_5(4.0, 1.0, 1.0);
	solution_path[4] = point_5;


	for (int i=0;i<10;i++) {
		Eigen::Vector3d s_point(1.0, i+1, 1.0);
		solution_path[i] = s_point;
	}


	std::map<std::string, std::shared_ptr<hdi_plan::Obstacle>> obstacle_map;
	std::string obstacle_name = "cube1";
	bool obstacle_operation = true;
	double obstacle_size = 1;
	Eigen::Vector3d obstacle_position(100, 100, 100);
	auto obstacle = std::make_shared<hdi_plan::Obstacle>(obstacle_name, hdi_plan::Obstacle_type::cube, obstacle_operation, obstacle_size, obstacle_position);
	obstacle_map[obstacle_name] = obstacle;

	ros::WallTime chomp_start_time = ros::WallTime::now();
	auto chomp_trajectory = std::make_shared<hdi_plan::ChompTrajectory>(solution_path);
	auto chomp = std::make_shared<hdi_plan::Chomp>(chomp_trajectory, obstacle_map);
	std::vector<Eigen::Vector3d> optimized_trajectory = chomp->get_optimized_trajectory();

	double chomp_process_time = (ros::WallTime::now() - chomp_start_time).toSec();
	std::cout << "The optimization time is: " << chomp_process_time << std::endl;

	int trajectory_size = optimized_trajectory.size();
	std::ofstream data_file ("data.txt");
	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d point = optimized_trajectory.at(i);
		//ROS_INFO("the optimized trajectory is: x=%.2f, y=%.2f, z=%.2f", point(0), point(1), point(2));
		//std::cout << point(0) << " " << point(1) << std::endl;
		data_file << point(0) << " " << point(1) << "\n";
	}
	if (data_file.is_open()) {
		data_file.close();
	}

	int trajectory_size = solution_path.size();

	hdi_plan::point_array trajectory_msg;
	geometry_msgs::Point trajectory_point;
	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d point = solution_path.at(i);
		trajectory_point.x = point(0);
		trajectory_point.y = point(1);
		trajectory_point.z = point(2);
		trajectory_msg.points.push_back(trajectory_point);
		//ROS_INFO("the optimized trajectory is: x=%.2f, y=%.2f, z=%.2f", point(0), point(1), point(2));
	}

	//std_msgs::Bool get_new_path_msg;
	//get_new_path_msg.data = true;
	//pub_get_new_path_.publish(get_new_path_msg);

	ROS_INFO("Publish the path now1");

	while (ros::ok()) {
		ROS_INFO("Start publish 1");
		ros::Duration(1.0).sleep();
		ROS_INFO("Start publish 2");
		pub_optimized_path_.publish(trajectory_msg);
		ROS_INFO("Start publish 3");
		ros::spinOnce();
		loop_rate.sleep();
	}



	return 0;
}
*/

/*
#include "generate_dynamic_scene/generate_obstacle.hpp"

namespace hdi_plan {

GenerateObstacle::GenerateObstacle(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
		: nh_(nh),
		  pnh_(pnh) {
	// load parameters
	if (!this->load_params()) {
		ROS_WARN("[%s] Could not load all parameters in motion planner.",
				 this->pnh_.getNamespace().c_str());
	} else {
		ROS_INFO("[%s] Loaded all parameters in motion planner.", this->pnh_.getNamespace().c_str());
	}
	ros::Duration(1.0).sleep();
	publish_trajectory();
	ROS_INFO("FInished");
}

GenerateObstacle::~GenerateObstacle() = default;


bool GenerateObstacle::load_params() {
	this->pnh_.getParam("total_planning_time", this->total_plan_time_);
	this->pnh_.getParam("quadrotor_radius", this->quadrotor_radius_);
	this->pnh_.getParam("quadrotor_speed", this->quadrotor_speed_);
	this->pnh_.getParam("collision_threshold", this->collision_threshold_);
	this->pnh_.getParam("planning_time_limit", this->planning_time_limit_);
	this->pnh_.getParam("max_iterations", this->max_iterations_);
	this->pnh_.getParam("max_iterations_after_collision_free", this->max_iterations_after_collision_free_);
	this->pnh_.getParam("learning_rate", this->learning_rate_);
	this->pnh_.getParam("obstacle_cost_weight", this->obstacle_cost_weight_);
	this->pnh_.getParam("dynamic_obstacle_cost_weight", this->dynamic_obstacle_cost_weight_);
	this->pnh_.getParam("dynamic_collision_factor", this->dynamic_collision_factor_);
	this->pnh_.getParam("smoothness_cost_weight", this->smoothness_cost_weight_);
	this->pnh_.getParam("smoothness_cost_velocity", this->smoothness_cost_velocity_);
	this->pnh_.getParam("smoothness_cost_acceleration", this->smoothness_cost_acceleration_);
	this->pnh_.getParam("smoothness_cost_jerk", this->smoothness_cost_jerk_);
	this->pnh_.getParam("ridge_factor", this->ridge_factor_);
	this->pnh_.getParam("min_clearence", this->min_clearence_);
	this->pnh_.getParam("joint_update_limit", this->joint_update_limit_);
	this->pnh_.getParam("discretization", this->discretization_);

	return true;
}

void GenerateObstacle::publish_trajectory() {
	std::vector<Eigen::Vector3d> solution_path;
	solution_path.resize(11);
	Eigen::Vector3d point_1(2.45292, 3.51743, 1.15095);
	solution_path[0] = point_1;
	Eigen::Vector3d point_2(3.67979, 5.12402, 1.01063);
	solution_path[1] = point_2;
	Eigen::Vector3d point_3(5.96799, 7.49909, 1.99399);
	solution_path[2] = point_3;
	Eigen::Vector3d point_4(6.16125, 9.16252, 2.19597);
	solution_path[3] = point_4;
	Eigen::Vector3d point_5(6.45006, 12.4731, 1.64777);
	solution_path[4] = point_5;
	Eigen::Vector3d point_6(9.49938, 13.6013, 1.80064);
	solution_path[5] = point_6;
	Eigen::Vector3d point_7(12.3138, 14.9923, 2.36284);
	solution_path[6] = point_7;
	Eigen::Vector3d point_8(14.7372, 17.1306, 1.78767);
	solution_path[7] = point_8;
	Eigen::Vector3d point_9(15.3523, 17.9035, 1.94358);
	solution_path[8] = point_9;
	Eigen::Vector3d point_10(17.815, 18.8867, 2.0752);
	solution_path[9] = point_10;
	Eigen::Vector3d point_11(20, 20, 2);
	solution_path[10] = point_11;

	
	std::vector<Eigen::Vector3d> solution_path;
	solution_path.resize(11);
	Eigen::Vector3d point_1(2.84782, 3.53338, 1.25199);
	solution_path[0] = point_1;
	Eigen::Vector3d point_2(3.548, 4.2471, 1.23326);
	solution_path[1] = point_2;
	Eigen::Vector3d point_3(5.7596, 6.63072, 1.24123);
	solution_path[2] = point_3;
	Eigen::Vector3d point_4(9.17822, 6.41438, 0.634579);
	solution_path[3] = point_4;
	Eigen::Vector3d point_5(10.4976, 6.69923, 0.639617);
	solution_path[4] = point_5;
	Eigen::Vector3d point_6(13.3017, 8.46523, 1.43444);
	solution_path[5] = point_6;
	Eigen::Vector3d point_7(14.3369, 10.4171, 0.835684);
	solution_path[6] = point_7;
	Eigen::Vector3d point_8(15.4546, 13.5029, 0.638429);
	solution_path[7] = point_8;
	Eigen::Vector3d point_9(17.4934, 15.2709, 1.26955);
	solution_path[8] = point_9;
	Eigen::Vector3d point_10(19.1406, 17.4181, 1.61743);
	solution_path[9] = point_10;
	Eigen::Vector3d point_11(20, 20, 2);
	solution_path[10] = point_11;

	
	std::vector<Eigen::Vector3d> solution_path;
	solution_path.resize(9);
	Eigen::Vector3d point_1(4.55281, 4.11356, 1.49805);
	solution_path[0] = point_1;
	Eigen::Vector3d point_2(3.78132, 6.6611, 1.01547);
	solution_path[1] = point_2;
	Eigen::Vector3d point_3(5.02077, 8.07318, 2.01937);
	solution_path[2] = point_3;
	Eigen::Vector3d point_4(5.65907, 9.77609, 1.20362);
	solution_path[3] = point_4;
	Eigen::Vector3d point_5(6.77827, 11.9087, 1.74742);
	solution_path[4] = point_5;
	Eigen::Vector3d point_6(7.67982, 15.0323, 1.52923);
	solution_path[5] = point_6;
	Eigen::Vector3d point_7(10.43, 17.0713, 1.81861);
	solution_path[6] = point_7;
	Eigen::Vector3d point_8(13.3463, 17.6473, 1.11981);
	solution_path[7] = point_8;
	Eigen::Vector3d point_9(16.4863, 18.2276, 1.60694);
	solution_path[7] = point_9;
	Eigen::Vector3d point_10(17.9467, 18.4052, 1.66529);
	solution_path[8] = point_10;
	Eigen::Vector3d point_11(20, 20, 2);
	solution_path[9] = point_11;
	

	std::ofstream data_file2("original_trajectory.txt");
	for (int i = 0; i < static_cast<int>(solution_path.size()); i++) {
		Eigen::Vector3d point = solution_path.at(i);
		data_file2 << point(0) << " " << point(1) << " " << point(2) << "\n";
	}
	if (data_file2.is_open()) {
		data_file2.close();
	}

	std::map<std::string, std::shared_ptr<hdi_plan::Obstacle>> obstacle_map;
	std::map<int, std::shared_ptr<Human>> human_map;
	std::string obstacle_name = "cube1";
	bool obstacle_operation = true;
	double obstacle_size = 3;
	Eigen::Vector3d obstacle_position(10.0, 10.0, 2.0);
	auto obstacle = std::make_shared<hdi_plan::Obstacle>(obstacle_name, hdi_plan::Obstacle_type::sphere, obstacle_operation, obstacle_size, obstacle_position);
	obstacle_map[obstacle_name] = obstacle;
	

	ros::WallTime chomp_start_time = ros::WallTime::now();

	auto chomp_trajectory = std::make_shared<ChompTrajectory>(solution_path, 1.0 , this->total_plan_time_, this->discretization_, this->quadrotor_speed_);
	auto chomp = std::make_shared<Chomp>(this->collision_threshold_, this->planning_time_limit_, this->max_iterations_,
									    this->max_iterations_after_collision_free_, this->learning_rate_, this->obstacle_cost_weight_, this->dynamic_obstacle_cost_weight_,
									    this->dynamic_collision_factor_, this->smoothness_cost_weight_, this->smoothness_cost_velocity_, this->smoothness_cost_acceleration_,
									    this->smoothness_cost_jerk_, this->ridge_factor_, this->min_clearence_, this->joint_update_limit_, this->quadrotor_radius_,
										chomp_trajectory, obstacle_map, human_map, 1);

	std::vector<Eigen::Vector3d> optimized_trajectory = chomp->get_optimized_trajectory();

	double chomp_process_time = (ros::WallTime::now() - chomp_start_time).toSec();
	std::cout << "The optimization time is: " << chomp_process_time << std::endl;


	std::ofstream data_file1("optimized_trajectory.txt");
	for (int i = 0; i < static_cast<int>(optimized_trajectory.size()); i++) {
		Eigen::Vector3d point = optimized_trajectory.at(i);
		std::cout << point(0) << " " << point(1) << " " << point(2) << std::endl;
		data_file1 << point(0) << " " << point(1) << " " << point(2) << "\n";
	}
	if (data_file1.is_open()) {
		data_file1.close();
	}
	}
}*/
