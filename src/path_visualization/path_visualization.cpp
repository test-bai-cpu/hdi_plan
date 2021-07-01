#include "path_visualization/path_visualization.hpp"

namespace hdi_plan {

PathVisualization::PathVisualization(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
		: nh_(nh),
		  pnh_(pnh) {
	trajectory_vis_pub_ = nh_.advertise<visualization_msgs::Marker>(
			"/visualization_marker", 1);
	trajectory_sub_ = nh_.subscribe("hdi_plan/full_trajectory", 1, &PathVisualization::trajectory_visualization_callback, this);
}

PathVisualization::~PathVisualization() = default;

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
	marker.id = this->marker_id_;
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



}