#include "path_visualization/path_visualization.hpp"

namespace hdi_plan {

PathVisualization::PathVisualization(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
		: nh_(nh),
		  pnh_(pnh) {

	// load parameters
	if (!this->load_params()) {
		ROS_WARN("[%s] Could not load all parameters in motion planner.",
				 this->pnh_.getNamespace().c_str());
	} else {
		ROS_INFO("[%s] Loaded all parameters in motion planner.", this->pnh_.getNamespace().c_str());
	}

	trajectory_vis_pub_ = nh_.advertise<visualization_msgs::Marker>(
			"/visualization_marker", 1);
	quadrotor_vis_pub_ = nh_.advertise<visualization_msgs::Marker>(
			"/visualization_marker", 1);
	path_spot_pub_ = nh_.advertise<hdi_plan::obstacle_info>("hdi_plan/path_spot_topic", 100);
	trajectory_sub_ = nh_.subscribe("hdi_plan/full_trajectory", 1, &PathVisualization::trajectory_callback, this);
	//trajectory_sub_ = nh_.subscribe("hdi_plan/full_trajectory", 1, &PathVisualization::trajectory_visualization_callback, this);
	node_position_sub_ = nh_.subscribe("hdi_plan/node_position", 1, &PathVisualization::node_position_callback, this);
	blocked_node_position_sub_ = nh_.subscribe("hdi_plan/blocked_node_position", 1, &PathVisualization::blocked_node_position_callback, this);
	quadrotor_state_sub_ = nh_.subscribe("hdi_plan/quadrotor_state", 1, &PathVisualization::quadrotor_state_callback, this);
}

PathVisualization::~PathVisualization() = default;

bool PathVisualization::load_params() {
	this->pnh_.getParam("goal_position", this->goal_position_param_);
	return true;
}

void PathVisualization::trajectory_callback(const hdi_plan::point_array::ConstPtr &msg) {
	int trajectory_size = msg->points.size();
	int path_spot_list_size = this->path_spot_list_.size();
	std::cout << "Size: " << trajectory_size << " spot: " << path_spot_list_size << std::endl;

	int index = 0;
	int stop_index = -1;
	for (auto &obstacle_msg : this->path_spot_list_) {
		if (index < trajectory_size) {
			obstacle_msg.position.x = msg->points[index].x;
			obstacle_msg.position.y = msg->points[index].y;
			obstacle_msg.position.z = msg->points[index].z;
			index += 1;
			continue;
		}

		if (obstacle_msg.position.x == 100.0 && obstacle_msg.position.y == 100.0 && obstacle_msg.position.z == 100.0) {
			stop_index = index;
			break;
		}

		obstacle_msg.position.x = 100.0;
		obstacle_msg.position.y = 100.0;
		obstacle_msg.position.z = 100.0;
		index += 1;
	}

	if (trajectory_size > path_spot_list_size) {
		for (int i = path_spot_list_size; i < trajectory_size; i++) {
			hdi_plan::obstacle_info obstacle_msg;
			obstacle_msg.name = "spot_" + std::to_string(i);
			obstacle_msg.type = hdi_plan::Obstacle_type::spot;
			obstacle_msg.operation = true;
			obstacle_msg.size = 0.1;
			obstacle_msg.position.x = msg->points[i].x;
			obstacle_msg.position.y = msg->points[i].y;
			obstacle_msg.position.z = msg->points[i].z;
			this->path_spot_list_.push_back(obstacle_msg);
		}
	}

	this->publish_path_spot(stop_index);
}

void PathVisualization::publish_path_spot(int stop_index) {
	int index = 0;
	for (auto obstacle_msg : this->path_spot_list_) {
		if (index == stop_index) break;
		//std::cout << "spot pos: " << obstacle_msg.position.x << " " << obstacle_msg.position.y << " " << obstacle_msg.position.z << std::endl;
		this->path_spot_pub_.publish(obstacle_msg);
		index += 1;
	}
}

visualization_msgs::MarkerArray PathVisualization::visualizationMarkers_to_msg(const VisualizationMarkers &markers) {
	visualization_msgs::Marker marker_msg;
	visualization_msgs::MarkerArray marker_array_msg;
	marker_array_msg.markers.clear();
	for (const VisualizationMarker &marker : markers.getMarkers()) {
		marker_msg.type = marker.type;
		marker_msg.action = marker.action;
		marker_msg.id = marker.id;
		marker_msg.ns = marker.ns;
		marker_msg.text = marker.text;
		marker_msg.pose.position.x = marker.position.x();
		marker_msg.pose.position.y = marker.position.y();
		marker_msg.pose.position.z = marker.position.z();
		marker_msg.pose.orientation.x = marker.orientation.x();
		marker_msg.pose.orientation.y = marker.orientation.y();
		marker_msg.pose.orientation.z = marker.orientation.z();
		marker_msg.pose.orientation.w = marker.orientation.w();
		marker_msg.scale.x = marker.scale.x();
		marker_msg.scale.y = marker.scale.y();
		marker_msg.scale.z = marker.scale.z();
		marker_msg.color.r = marker.color.r;
		marker_msg.color.g = marker.color.g;
		marker_msg.color.b = marker.color.b;
		marker_msg.color.a = marker.color.a;
		marker_msg.points.clear();
		geometry_msgs::Point point;
		for (const Eigen::Vector3d &p : marker.points) {
			point.x = p.x();
			point.y = p.y();
			point.z = p.z();
			marker_msg.points.push_back(point);
		}
		marker_msg.colors.clear();
		std_msgs::ColorRGBA color;
		for (const Color &c : marker.colors) {
			color.r = c.r;
			color.g = c.g;
			color.b = c.b;
			color.a = c.a;
			marker_msg.colors.push_back(color);
		}
		marker_array_msg.markers.push_back(marker_msg);
	}

	return marker_array_msg;
}

visualization_msgs::Marker PathVisualization::visualizationMarker_to_msg(const VisualizationMarker &marker) {
	visualization_msgs::Marker marker_msg;
	marker_msg.header.frame_id = "world";
	marker_msg.header.stamp = ros::Time::now();
	marker_msg.type = marker.type;
	marker_msg.action = marker.action;
	marker_msg.ns = "basic_shapes";
	marker_msg.id = marker.id;
	marker_msg.ns = marker.ns;
	marker_msg.text = marker.text;
	marker_msg.pose.position.x = marker.position.x();
	marker_msg.pose.position.y = marker.position.y();
	marker_msg.pose.position.z = marker.position.z();
	marker_msg.pose.orientation.x = 0.0;
	marker_msg.pose.orientation.y = 0.0;
	marker_msg.pose.orientation.z = 0.0;
	marker_msg.pose.orientation.w = 1.0;
	marker_msg.scale.x = 0.1;
	marker_msg.scale.y = 0.1;
	marker_msg.scale.z = 0.1;
	marker_msg.color.r = marker.color.r;
	marker_msg.color.g = marker.color.g;
	marker_msg.color.b = marker.color.b;
	marker_msg.color.a = marker.color.a;
	marker_msg.points.clear();

	for (const Eigen::Vector3d &p : marker.points) {
		geometry_msgs::Point point;
		point.x = p.x();
		point.y = p.y();
		point.z = p.z();
		marker_msg.points.push_back(point);
	}
	marker_msg.colors.clear();
	std_msgs::ColorRGBA color;
	for (const Color &c : marker.colors) {
		color.r = c.r;
		color.g = c.g;
		color.b = c.b;
		color.a = c.a;
		marker_msg.colors.push_back(color);
	}
	marker_msg.lifetime = ros::Duration();
	return marker_msg;
}

void PathVisualization::publish_visualization(const VisualizationMarker& marker) {
	visualization_msgs::Marker marker_msg = this->visualizationMarker_to_msg(marker);
	this->trajectory_vis_pub_.publish(marker_msg);
}

void PathVisualization::trajectory_visualization_callback(const hdi_plan::point_array::ConstPtr &msg) {
	int trajectory_size = msg->points.size();
	VisualizationMarker marker;
	marker = VisualizationMarker();
	marker.orientation.w() = 1.0;
	marker.type = VisualizationMarker::POINTS;
	marker.id = 0;
	marker.ns = "candidate_trajectories";
	marker.scale.x() = 0.04;
	marker.scale.y() = 0.04;
	marker.color.a = 0.4;
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.action = VisualizationMarker::ADD;


	for (int i = 0; i < trajectory_size; i++) {
		Eigen::Vector3d p(msg->points[i].x, msg->points[i].y, msg->points[i].z);
		marker.points.push_back(p);
	}

	this->publish_visualization(marker);
}


void PathVisualization::node_position_callback(const geometry_msgs::Point::ConstPtr &msg) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = this->marker_id_;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = msg->x;
	marker.pose.position.y = msg->y;
	marker.pose.position.z = msg->z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 0.4;
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;

	this->trajectory_vis_pub_.publish(marker);

	this->marker_id_ += 1;
}

void PathVisualization::quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 1;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = msg->pose.pose.position.x;
	marker.pose.position.y = msg->pose.pose.position.y;
	marker.pose.position.z = msg->pose.pose.position.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.color.a = 0.4;
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;

	this->quadrotor_vis_pub_.publish(marker);
}

void PathVisualization::blocked_node_position_callback(const geometry_msgs::Point::ConstPtr &msg) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = this->blocked_node_marker_id_;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = msg->x;
	marker.pose.position.y = msg->y;
	marker.pose.position.z = msg->z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 0.4;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;

	this->trajectory_vis_pub_.publish(marker);

	this->blocked_node_marker_id_ += 1;
}


}