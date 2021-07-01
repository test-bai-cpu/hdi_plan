#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <hdi_plan/point_array.h>

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

	ros::Publisher trajectory_vis_pub_;

	int marker_id_{0};

	void publish_visualization(const VisualizationMarker& marker);
	void trajectory_visualization_callback(const hdi_plan::point_array::ConstPtr &msg);
	visualization_msgs::Marker visualizationMarker_to_msg(const VisualizationMarker &marker);
	visualization_msgs::MarkerArray visualizationMarkers_to_msg(const VisualizationMarkers &markers);
};
}