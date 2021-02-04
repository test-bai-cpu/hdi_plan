#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
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

    return 0;
}
