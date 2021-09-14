#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include "generate_dynamic_scene/obstacle.hpp"
#include "generate_dynamic_scene/human.hpp"
#include <hdi_plan/obstacle_info.h>
#include <hdi_plan/point_array.h>
#include "utils/utility_functions.hpp"

#include "path_visualization/visualization_markers.hpp"

namespace hdi_plan {

class PathVisualization {
public:
	PathVisualization(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
	~PathVisualization();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Subscriber trajectory_sub_;
	ros::Subscriber node_position_sub_;
	ros::Subscriber quadrotor_state_sub_;
	ros::Subscriber blocked_node_position_sub_;
	ros::Publisher trajectory_vis_pub_;
	ros::Publisher quadrotor_vis_pub_;
	ros::Publisher path_spot_pub_;

	int marker_id_{2};
	int blocked_node_marker_id_{0};

	void publish_visualization(const VisualizationMarker& marker);
	void trajectory_visualization_callback(const hdi_plan::point_array::ConstPtr &msg);
	void node_position_callback(const geometry_msgs::Point::ConstPtr &msg);
	void blocked_node_position_callback(const geometry_msgs::Point::ConstPtr &msg);
	void quadrotor_state_callback(const nav_msgs::Odometry::ConstPtr &msg);
	visualization_msgs::Marker visualizationMarker_to_msg(const VisualizationMarker &marker);
	visualization_msgs::MarkerArray visualizationMarkers_to_msg(const VisualizationMarkers &markers);

	bool load_params();
	void trajectory_callback(const hdi_plan::point_array::ConstPtr &msg);
	void publish_path_spot(int stop_index);

	std::vector<hdi_plan::obstacle_info> path_spot_list_;
	std::vector<double> goal_position_param_;
};
}