#include <ros/ros.h>
#include<geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include "mainpulator/mainpulator_trajectory.h"
using namespace std;
double q = 0;
double qd = 0;
        double qdd = 0;
        double t = 0;
        double at = 0.5;
		vector<double>  expect_q;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "TrajectoryNode");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("trajectory", 100);
	ros::Rate loop_rate(500);
    double time = 0;
	double q = 0;
    while (ros::ok)
    {
        expect_q =  trajectory::p_to_p(time);
	    sensor_msgs::JointState joint_state;
	    joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(5);
		joint_state.position.resize(5);
		joint_state.velocity.resize(5);
		joint_state.effort.resize(5);
        // 位置
		/*joint_state.position[1] = qd;
		joint_state.position[2] = qdd;
		joint_state.position[3] = t;
		joint_state.position[4] = 4;*/
		joint_state.position[0] = expect_q[0];
		joint_state.position[1] = q;
		q += 0.2;
		//速度
		/*joint_state.velocity[1] = 110;
		joint_state.velocity[2] = 11;
		joint_state.velocity[3] = 13;
		joint_state.velocity[4] = 4;*/
		joint_state.velocity[0] = expect_q[1];
		//加速度
		/*joint_state.effort[1] = 3;
		joint_state.effort[2] = 1;
		joint_state.effort[3] = 6;
		joint_state.effort[4] = 4;*/
		joint_state.effort[0] = expect_q[2];
		
		
		joint_state.effort[1] = 2;
	  //  ROS_INFO("q = %lf",expect_q[0]);


        chatter_pub.publish(joint_state);
		time +=0.002;
		if(time>20) time-=20;
		ros::spinOnce();

		loop_rate.sleep();
    }
    return 0;
}
