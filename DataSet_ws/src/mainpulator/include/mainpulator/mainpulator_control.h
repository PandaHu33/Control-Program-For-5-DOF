#ifndef H_MAINPULATOR_CONTROL
#define H_MAINPULATOR_CONTROL
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/String.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>
#include <iostream>
#include <string>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>


#include <cmath>


using namespace std;
namespace control{
    class mainpulator
    {
        private:
            /* data */
            int id ;
            int position;
            double angleinit;
            double angle;
            int electricinit;
            double electric;
            double velocity = 0;
        public:
            //Mainpulator(int id);
            mainpulator(int id);
            mainpulator(int id,int position);
            ~mainpulator();
            //返回机械臂关节id
            int getid() {
                return this->id;
            };
            /// @brief 机械臂期望角度
            double  get_excpt_angle()
            {
                return this->angleinit;
            };
            /// @brief  机械臂当前角度
            double  get_current_angle()
            {
               // ROS_INFO("joint%d , curent_angle = %f ",this->id,this->angle);
                return this->angle;
            };
            double  get_current_velocity()
            {
                //ROS_INFO("joint%d , velocity = %f ",this->id,this->velocity);
                return this->velocity;
            };
            double get_ActualCurrent()
            {
                //ROS_INFO("joint5, electric = %lf ",this->electric);
                return this->electric;

            };
            void Position_init(const struct can_frame frames);
            void Position_init_val_for_joint5(const struct can_frame frames);
            void Electric_init(const struct can_frame frames);
            //void set_expect_q();
            
            can_msgs::Frame set_angle(double exp_angle);
            can_msgs::Frame set_angle_for_new_joint(double exp_angle);
            can_msgs::Frame MomentOutput(double tol);
            can_msgs::Frame MomentOutput80(double tol);

            void current_angle(const can_msgs::Frame& position);
            void current_velocity(const can_msgs::Frame& velocity);

            can_msgs::Frame ret_to_init(int positon);

            can_msgs::Frame ret_to_init();
            
            void ActualCurrent(const can_msgs::Frame& electric);

    };

    double arc_control(double qd,double dqd,double ddqd,double q,double dq,double t ,double m,double c,double g);

}

























#endif

