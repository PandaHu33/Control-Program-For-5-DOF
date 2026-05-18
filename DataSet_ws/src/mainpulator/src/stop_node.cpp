#include <ros/ros.h>
#include<geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <std_msgs/Bool.h>
int main(int argc, char** argv)
{
	ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("reboot_stop",10);
    std_msgs::Bool stop;
    ros::Rate rate_sleep(100);
    while(ros::ok)
    {
        stop.data = true;
        pub.publish(stop);
        ROS_INFO("send");
        ros::spinOnce();
        rate_sleep.sleep();
    }
}