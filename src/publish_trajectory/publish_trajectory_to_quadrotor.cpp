#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <hdi_plan/point_array.h>
#include <hdi_plan/obstacle_info.h>



void trajectory_callback(const hdi_plan::point_array::ConstPtr &msg) {
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "publish_trajectory_to_quadrotor");
	ros::NodeHandle nh;

	ros::Publisher go_to_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("autopilot/pose_command", 1);
	ros::Publisher max_velocity_pub = nh.advertise<std_msgs::Float64>("autopilot/max_velocity", 1);
	ros::Subscriber trajectory_sub = nh.subscribe("hdi_plan/full_trajectory", 1, trajectory_callback);
	ros::Publisher trajectory_pub = nh.advertise<hdi_plan::point_array>("hdi_plan/full_trajectory", 1);

	hdi_plan::point_array msg;
	geometry_msgs::Point trajectory_point;
	for (int i = 0; i < 10; i++) {
		trajectory_point.x = i;
		trajectory_point.y = i+1;
		msg.points.push_back(trajectory_point);
		trajectory_pub.publish(msg);
	}

	ros::spin();
	return 0;
}

