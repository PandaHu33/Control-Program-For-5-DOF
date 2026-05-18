#ifndef H_MAINPULATOR_TRAJECTORY
#define H_MAINPULATOR_TRAJECTORY
#include <ros/ros.h>
#include<geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <iostream>
#include <vector>
using namespace std;

namespace trajectory
{
    
       vector<double> p_to_p(double time);
  
} // namespace trajectory








#endif