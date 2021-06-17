#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <hdi_plan/point_array.h>

visualization_msgs::Marker marker;
ros::Publisher marker_pub;
int id = 0;

void trajectory_callback(const hdi_plan::point_array::ConstPtr &msg) {
	ROS_INFO("In the trajectory callback");
	int trajectory_size = msg->points.size();

	for (int i = 0; i < trajectory_size; i++) {
		std::cout << "show the pos in rviz, i = " << i << std::endl;

		ros::Duration(1).sleep();
		marker.id = 0;
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = msg->points[i].x;
		marker.pose.position.y = msg->points[i].y;
		marker.pose.position.z = msg->points[i].z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		marker_pub.publish(marker);
		id += 1;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle nh;
	//ros::Rate r(1);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber trajectory_sub = nh.subscribe("hdi_plan/full_trajectory", 1, trajectory_callback);
	// Set our initial shape type to be a cube
	ROS_INFO("Prepare the marker");
	uint32_t shape = visualization_msgs::Marker::CUBE;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "my_frame";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETE
	// ALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	ROS_INFO("Prepare the marker2");
	ros::spin();
	return 0;
}

/*
    rosgraph_msgs::Clock ros_time;
    double begin = ros::Time::now().toSec();
    ros::Publisher chatter_pub = n.advertise<rosgraph_msgs::Clock>("clock", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) {
        ROS_INFO("******update time**********");
        double currentTime = ros::Time::now().toSec();
        ros_time.clock.fromSec(currentTime-begin);
        chatter_pub.publish(ros_time);
        ros::spinOnce();
        loop_rate.sleep();

        ++count;
    }
 */