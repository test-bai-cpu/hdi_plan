#include "generate_dynamic_scene/generate_obstacle.hpp"

namespace hdi_plan {

GenerateObstacle::GenerateObstacle(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
		: nh_(nh),
		  pnh_(pnh) {
	this->human_movement_pub_ = nh_.advertise<geometry_msgs::Point>("hdi_plan/human_movement", 1);
	this->update_human_obstacle_pub_ = nh_.advertise<hdi_plan::obstacle_info>("hdi_plan/obstacle_info_topic", 1);

	// wait until human movement start
	//ros::Duration(25.0).sleep();
	//this->publish_human_movement();
	//ros::Duration(21.0).sleep();
	//this->publish_obstacle();
}

GenerateObstacle::~GenerateObstacle() = default;

hdi_plan::obstacle_info GenerateObstacle::get_obstacle_message(bool operation, double position_x, double position_y) {
	hdi_plan::obstacle_info obstacle_msg;
	obstacle_msg.name = "human";
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
	obstacle_msg.name = "cube";
	obstacle_msg.type = hdi_plan::Obstacle_type::cube;
	obstacle_msg.operation = true;
	obstacle_msg.size = 2;
	obstacle_msg.position.x = 10.0;
	obstacle_msg.position.y = 3.0;
	obstacle_msg.position.z = 1.0;
	update_human_obstacle_pub_.publish(obstacle_msg);
}

void GenerateObstacle::publish_human_movement() {
	Eigen::Vector2d start_point(5.0, 0.0);
	Eigen::Vector2d goal_point(5.0, 10.0);
	Eigen::Vector2d current_point(start_point(0), start_point(1));
	double distance = hdi_plan_utils::get_distance_2d(start_point, goal_point);
	double velocity = 1;
	double period = 1;

	int count = 0;
	ROS_INFO("#################Generate Obstacle: Start to spawn a human.");
	while (hdi_plan_utils::get_distance_2d(current_point, goal_point) > 0.5 * period * velocity) {
		if (count > 0) update_human_obstacle_pub_.publish(get_obstacle_message(false));

		current_point(0) = (goal_point(0) - start_point(0)) * (velocity * count * period) / distance + start_point(0);
		current_point(1) = (goal_point(1) - start_point(1)) * (velocity * count * period) / distance + start_point(1);

		//std::cout << "The human current position is, x: " << current_point(0) << " y: " << current_point(1) << std::endl;

		update_human_obstacle_pub_.publish(get_obstacle_message(true, current_point(0), current_point(1)));
		geometry_msgs::Point msg;
		msg.x = current_point(0);
		msg.y = current_point(1);
		msg.z = 0;
		human_movement_pub_.publish(msg);

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
	this->human_movement_pub_ = nh_.advertise<geometry_msgs::Point>("hdi_plan/human_movement", 1);
	this->update_human_obstacle_pub_ = nh_.advertise<hdi_plan::obstacle_info>("hdi_plan/obstacle_info_topic", 1);
	this->pub_optimized_path_ = nh_.advertise<hdi_plan::point_array>("hdi_plan/full_trajectory", 1);
	this->pub_get_new_path_ = nh_.advertise<std_msgs::Bool>("hdi_plan/get_new_path", 1);
	ros::Duration(25.0).sleep();
	this->publish_trajectory();
}

GenerateObstacle::~GenerateObstacle() = default;

void GenerateObstacle::publish_trajectory() {

	std::vector<Eigen::Vector3d> solution_path;
	solution_path.resize(11);
	Eigen::Vector3d point_1(0.0, 4.0, 1.0);
	solution_path[0] = point_1;
	Eigen::Vector3d point_2(1.0, 4.0, 1.0);
	solution_path[1] = point_2;
	Eigen::Vector3d point_3(2.0, 4.0, 1.0);
	solution_path[2] = point_3;
	Eigen::Vector3d point_4(3.0, 4.0, 1.0);
	solution_path[3] = point_4;
	Eigen::Vector3d point_5(4.0, 4.0, 1.0);

	solution_path[4] = point_5;
	Eigen::Vector3d point_6(5.0, 4.0, 1.0);
	solution_path[5] = point_6;
	Eigen::Vector3d point_7(6.0, 4.0, 1.0);
	solution_path[6] = point_7;
	Eigen::Vector3d point_8(7.0, 4.0, 1.0);
	solution_path[7] = point_8;
	Eigen::Vector3d point_9(8.0, 4.0, 1.0);
	solution_path[8] = point_9;
	Eigen::Vector3d point_10(9.0, 4.0, 1.0);
	solution_path[9] = point_10;
	Eigen::Vector3d point_11(10.0, 4.0, 1.0);
	solution_path[10] = point_11;

	std::vector<Eigen::Vector3d> test_path;
	test_path.resize(1);
	Eigen::Vector3d point_test(0.0, 4.0, 1.0);
	test_path[0] = point_test;


	std::map<std::string, std::shared_ptr<hdi_plan::Obstacle>> obstacle_map;
	std::string obstacle_name = "cube1";
	bool obstacle_operation = true;
	double obstacle_size = 0.5;
	Eigen::Vector3d obstacle_position(4.0, 4.0, 1.0);
	//Eigen::Vector3d obstacle_position(100.0, 100.0, 100);
	auto obstacle = std::make_shared<hdi_plan::Obstacle>(obstacle_name, hdi_plan::Obstacle_type::cube,
														 obstacle_operation, obstacle_size, obstacle_position);
	obstacle_map[obstacle_name] = obstacle;

	ros::WallTime chomp_start_time = ros::WallTime::now();
	auto chomp_trajectory = std::make_shared<hdi_plan::ChompTrajectory>(solution_path);
	auto chomp = std::make_shared<hdi_plan::Chomp>(chomp_trajectory, obstacle_map);
	std::vector<Eigen::Vector3d> optimized_trajectory = chomp->get_optimized_trajectory();

	double chomp_process_time = (ros::WallTime::now() - chomp_start_time).toSec();
	std::cout << "The optimization time is: " << chomp_process_time << std::endl;



	int trajectory_size = optimized_trajectory.size();
	std::ofstream data_file("data.txt");
	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d point = optimized_trajectory.at(i);
		//ROS_INFO("the optimized trajectory is: x=%.2f, y=%.2f, z=%.2f", point(0), point(1), point(2));
		//std::cout << point(0) << " " << point(1) << std::endl;
		data_file << point(0) << " " << point(1) << "\n";
	}
	if (data_file.is_open()) {
		data_file.close();
	}


	ROS_INFO("Start to publish the trajectory message.");

	for (int j = 0; j < 1; j++) {
		hdi_plan::point_array trajectory_msg;
		int trajectory_size = test_path.size();
		geometry_msgs::Point trajectory_point;
		for (int i = 0; i < trajectory_size; i++) {
			Eigen::Vector3d point = test_path.at(i);
			trajectory_point.x = 0.0;
			trajectory_point.y = 4.0;
			trajectory_point.z = 1.0;
			trajectory_msg.points.push_back(trajectory_point);
			//ROS_INFO("publish trajectory is: x=%.2f, y=%.2f, z=%.2f", point(0), point(1), point(2));
		}
		//ros::Duration(5.0).sleep();
		ROS_INFO("###Pub a new path");
		std::cout << "The trajectory size is " << trajectory_msg.points.size() << std::endl;
		pub_optimized_path_.publish(trajectory_msg);
	}
	}
}
*/