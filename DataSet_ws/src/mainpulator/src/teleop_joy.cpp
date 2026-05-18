#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include<iostream>
#include <std_msgs/Float64.h>
using namespace std;
class wheeltec_joy
{
public:
    wheeltec_joy();
private:
    void callback(const sensor_msgs::Joy::ConstPtr& Joy); 
    //实例化节点
    ros::NodeHandle n; 
    ros::Subscriber sub ;
    ros::Publisher pub ;
    double flag_A,flag_B,flag_C,flag_D,flag_E,flag_F;
};

wheeltec_joy::wheeltec_joy() 
{
   ros::NodeHandle private_nh("~"); //创建节点句柄

   pub = n.advertise<geometry_msgs::Twist>("joynode_transfer_pub",1);//将速度发给机器人底盘节点
   sub = n.subscribe<sensor_msgs::Joy>("joy",10,&wheeltec_joy::callback,this); //订阅手柄发来的数据
} 



void wheeltec_joy::callback(const sensor_msgs::Joy::ConstPtr& Joy) //键值回调函数
 {
   double JOY_LEFT,JOY_AHEAD,JOY_UP,JOY_CLOSE,JOY_OPEN,JOY_SPEEDUP;
      double JOY_TL,JOY_TR;
   //double acce_x,acce_z;
   geometry_msgs::Twist v;
   JOY_LEFT =Joy->axes[0];  //获取axes[0]的值
   JOY_AHEAD =Joy->axes[1];  //获取axes[1]的值
   JOY_UP=Joy->axes[2];  //获取axes[2]的值
   JOY_CLOSE=Joy->buttons[4];  //获取axes[3]的值
    JOY_OPEN=Joy->buttons[5];  //获取axes[3]的值

    JOY_SPEEDUP=Joy->buttons[6];  
    JOY_TL=Joy->buttons[3];  
    JOY_TR=Joy->buttons[1];  
   //判断前进后退
   if(JOY_AHEAD>0) 
   {
       flag_A=1;
   }
   else if(JOY_AHEAD<0)
   {
      flag_A=-1;
   }
   else  
   {
    flag_A=0;
   }

   if(JOY_LEFT>0) 
   {
       flag_B=1;
   }
   else if(JOY_LEFT<0)
   {
      flag_B=-1;
   }
   else  
   {
    flag_B=0;
   }

   if(JOY_UP>0) 
   {
       flag_C=1;
   }
   else if(JOY_UP<0)
   {
      flag_C=-1;
   }
   else  
   {
    flag_C=0;
   }


   if(JOY_TL>0&&JOY_TR==0) 
   {
       flag_F=1;
   }
   else if(JOY_TL==0&&JOY_TR>0)
   {
      flag_F=-1;
   }
   else  
   {
    flag_F=0;
   }


      if(JOY_SPEEDUP>0) 
   {
       flag_E=1;
   }
   else  
   {
        flag_E=0;
   }

   if(JOY_CLOSE>0&&JOY_OPEN==0) 
   {
       flag_D=1;
   }
   else if(JOY_CLOSE==0&&JOY_OPEN>0)
   {
      flag_D=-1;
   }
   else  
   {
    flag_D=0;
   }
   
    v.linear.x = flag_A*1.0;
    v.linear.y=  -flag_B*1.0;
    v.linear.z = -flag_C*1.0;
    v.angular.x = flag_D*1.0;
    v.angular.y = flag_E*1.0;
    v.angular.z = flag_F*1.0;
   ROS_INFO("XYZD_JOY: %lf,%lf,%lf,%lf,%lf,%lf",v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z);
   pub.publish(v);
}

int main(int argc,char** argv)
{
  ros::init(argc, argv, "joy_control");
  wheeltec_joy teleop_JOY_1;
  ros::spin();
  return 0;

} 