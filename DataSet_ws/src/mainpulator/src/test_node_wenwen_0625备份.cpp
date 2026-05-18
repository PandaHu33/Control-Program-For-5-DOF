#include "mainpulator/mainpulator_param.h"
#include "mainpulator/mainpulator_control.h"
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
//用到传9消息,需要一个合适的消息类型
#include <sensor_msgs/Imu.h>

#define PI acos(-1)
using namespace std;
using namespace Eigen;
string control_type;
control::mainpulator joint1(1, 262144); //pi   //12509
control::mainpulator joint2(2, 262144); //pi
control::mainpulator joint3(3, 236009); //196608//0.75pi   //236009//367081//新机械臂327680向下，65536向上

control::mainpulator joint4(4, 262144);
control::mainpulator joint5(5, 2000);
std_msgs::Bool stop_flag;


// 抓手（关节5）和关节4相关变量定义
double KB_D = 0.0;
double tol5 = 0.0;
double joint4angle = 0.0001; //实际上是关节4期望角度
double joint4_actual_angle = 0.0001;
double joint5_actual_angle = 0.0;
double joint4_actual_velocity = 0.0;
double joint5_actual_current = 0.0;


//电动机械臂自身参数和变量
double m1 = 9.6;
double m2  = 6.4;
double l1 = 0.424;
double l2 = 0.268;
Matrix3Xd Mn(3,3);
Matrix3Xd Cn(3,3);
Vector3d Gn;

//g根据安装方向修改 橙色底座为-------
double g = 9.8;
//实际角度
Vector3d q;
//实际角速度
Vector3d dq;
//期望角度_主端传过来的角度qm
Vector3d expect_q;
//期望角速度_主端传过来的角速度dqm
Vector3d expect_dq;
//期望角加速度_主端传过来的角加速度度ddqm
Vector3d expect_ddq;

double expect_q4 =0.0001;  //关节4期望角度

Vector3d tol;

// AdaptiveBackstepping参数表
Matrix3d lambda1;
Matrix3d lambda2;
Matrix3d kexi;
VectorXd theta_d(11);
VectorXd theta(11);
VectorXd theta_max(11);
VectorXd theta_min(11);
VectorXd slidetheta(11);
Vector3d u;
MatrixXd gammamamm(11,11);
//Eigen::Matrix<double, 2, 8> fai_Te;  
MatrixXd fai(3,11);
//Eigen::Matrix<double, 2, 8> fai_T;  
//MatrixXd fai_dot(3,11);
//Matrix3Xd theta_max1 (3,3);
//Matrix3Xd Xite (3,3);

// 定义一些中间变量
Vector3d alpha1;
Vector3d dalpha1;
//关节误差
Vector3d z1;
Vector3d dz1;
Vector3d z2;
Vector3d s; //滑膜量

// 摩擦参数
Vector3d fc;
Vector3d fv;

// NDOB参数和相关变量
Matrix3d L;
Vector3d xi;
Vector3d xi_dot;
Vector3d d_hat;


Vector3d zerovector;

void  AdaptiveBACKSTEPPINGparam_init();
void  ForceObserverparam_init();


Vector3d AdaptiveBackstepping(Vector3d& expect_q, Vector3d& expect_dq, Vector3d& expect_ddq, Vector3d& q, Vector3d& dq, Vector3d& tol_e_hat, double t);
Vector3d NDOB(Vector3d& tol,Vector3d& q,Vector3d& dq,double t);

Vector3d sgns(Vector3d& s);

void MainpulatorCallback(const can_msgs::Frame &receive_message);
/** 
 * @brief 位置模式回调函数,编码器返回速度、位置等信息
 * @param receive_message ros_canopen类对象
 * @return 
 */

void TeleOperationCallback(const  sensor_msgs::Imu& msg);



int main(int argc, char *argv[])
{
    // demoKinematics();
    //实际角度初始化
    q(0, 0) = 0.00001;
    q(1, 0) = 0.00001;
    q(2, 0) = 0.00001;
    //执行 ros 节点初始化
    ros::init(argc, argv, "mainpulator_param_node");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle nh("~");
    ros::NodeHandle socketcan_send;
    ros::NodeHandle socketcan_receive;
    ros::NodeHandle receive_stop;
    ros::NodeHandle trajectory_receive;
    ros::NodeHandle mainpulator_receive_sub;
    ros::Subscriber socketcan_receive_sub;

    stop_flag.data = false;
    //机械臂控制方式 位置控制 力矩控制
    nh.getParam("control_type", control_type);
    //控制器发布数据至机械臂
    ROS_INFO("control_type = %s", control_type.c_str());
    ros::Publisher socketcan_send_pub = socketcan_send.advertise<can_msgs::Frame>("sent_messages", 10);

    //控制接收机械臂数据
    socketcan_receive_sub = socketcan_receive.subscribe("received_messages", 1000, MainpulatorCallback);


    // 订阅器TeleOperation_receive_sub订阅遥操作期望信息，话题为"/pub_joint_state"，回调函数为TeleOperationCallback
    ros::NodeHandle TeleOperation;
    ros::Subscriber TeleOperation_receive_sub;
    TeleOperation_receive_sub= TeleOperation.subscribe("/pub_joint_state",10, TeleOperationCallback);  

    ros::Publisher TeleOperation_pub;
    TeleOperation_pub = TeleOperation.advertise<sensor_msgs::JointState>("/chatter",10);
     
    expect_q(0,0) = 0.00001;
    expect_q(1,0) = 0.00001;
    expect_q(2,0) = 0.00001;

    expect_dq(0,0) = 0.00001;
    expect_dq(1,0) = 0.00001;
    expect_dq(2,0) = 0.00001;

    expect_ddq(0,0) = 0.00001;
    expect_ddq(1,0) = 0.00001;
    expect_ddq(2,0) = 0.00001;

    zerovector(0,0)=0.00001;
    zerovector(1,0)=0.00001;
    zerovector(2,0)=0.00001;


    ros::Rate loop_rate(100);
    int socket_can = param::SocketCANInit();
    ROS_INFO("socket_can = %d", socket_can);
    ROS_INFO("control_type  =  %s", control_type.c_str());

    can_msgs::Frame send_message;

    //机械臂输出化配置
    ROS_INFO("init");
    param::MomentInit(joint5, socket_can);
    param::MomentInit(joint1, socket_can);
    param::MomentInit(joint2, socket_can);
    param::MomentInit(joint3, socket_can);
    param::PositionInit(joint4, socket_can);
    // param::PositionInit(joint5,socket_can);
    usleep(500000);
    //机械臂使能
    //param::Enable(socket_can, 6, joint1);
    //param::Enable(socket_can, 6, joint2);
    //param::Enable(socket_can, 6, joint3);
    //param::Enable(socket_can, 6, joint4);
    //param::Enable(socket_can, 6, joint5);
    usleep(500000);
    ROS_INFO("init end");

    AdaptiveBACKSTEPPINGparam_init();
    ForceObserverparam_init();

    // 摩擦参数初始化
    fc(0,0)  =  7.65;
    fc(1,0)  = 7.738;
    fc(2,0) =  6.319; 
    fv(0,0)  =  19.6233;
    fv(1,0)  = 13.758;
    fv(2,0) = 12.865; 

    while (ros::ok)
    {
        static long run_times = 0;
        if (run_times++ % 100 == 0)
        {
            cout << "\033c";
        }
        // cout << "\033[2J\033[1;1H";
        cout << "\033[0;0H";

        // ROS_INFO("Joint1_Angle = %lf", q(0,0));
        //ROS_INFO("Joint2_Angle = %lf", q(1,0));
        //处理排队的回调函数
        ros::spinOnce();
        
        // 发布给主端角度、角速度、估计外力
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.position.resize(8);
        joint_state.velocity.resize(8);
        joint_state.effort.resize(13);
            
        joint_state.position[0] = expect_q(0,0);
        joint_state.position[1] = expect_q(1,0);
        joint_state.position[2] = expect_q(2,0);
        joint_state.position[3] = q(0,0);
        joint_state.position[4] = q(1,0);
        joint_state.position[5] = q(2,0);
        //joint_state.position[6] = Gripper_flag;
        //joint_state.position[7] = Homing_flag;

        joint_state.velocity[0] = expect_dq(0,0);
        joint_state.velocity[1] = expect_dq(1,0);
        joint_state.velocity[2] = expect_dq(2,0);
        joint_state.velocity[3] = dq(0,0); 
        joint_state.velocity[4] = dq(1,0);  
        joint_state.velocity[5] = dq(2,0);   
        //joint_state.velocity[6] = d_hat(0,0);
        //joint_state.velocity[7] = d_hat(1,0);
        //joint_state.effort[0] = d_hat(2,0);

    
        joint_state.effort[1] = tol(0,0);
        joint_state.effort[2] = tol(1,0);
        joint_state.effort[3] = tol(2,0);
        //joint_state.effort[4] = joint5_actual_angle;   // 抓手实际张合距离
        //joint_state.effort[5] = joint5_actual_current; //抓手电流
        joint_state.effort[4] = joint4angle;   // 
        joint_state.effort[5] = joint4_actual_angle; //
        //joint_state.effort[6] = TelMappingMode_flag;
        //joint_state.effort[7] = MoveMode_flag;
        //joint_state.effort[8] = theta(0,0);//-PID_D(2,2)*derr(2,0);
        //joint_state.effort[9]= theta(1,0);//u(2,0);
        //joint_state.effort[10]= -fai_dot(1,1)*theta(1,0);//tol_model(2,0);
        // joint_state.effort[11]= -fai_dot(1,0)*theta(0,0);
        //joint_state.effort[12]= theta(9,0);
        //joint_state.effort[6] = joint5_actual_angle;

        TeleOperation_pub.publish(joint_state);
        tol=AdaptiveBackstepping(expect_q,expect_dq,expect_ddq,q,dq,zerovector,0.002);
        
        d_hat = NDOB(tol,q,dq,0.002);
        ROS_INFO("d_hat1 = %lf d_hat2 = %lf d_hat3 = %lf ",d_hat(0,0),d_hat(1,0),d_hat(2,0));

        if(tol(0,0)>30)
            {
                tol(0,0)=30;
            }
        else if(tol(0,0)<-30)
            {
                tol(0,0)=-30;
            }

        if(tol(1,0)>100)
            {
                tol(1,0)=100;
            }
        else if(tol(1,0)<-100)
            {
                tol(1,0)=-100;
            }
        if(tol(2,0)>35)
            {
                tol(2,0)=35;
            }
        else if(tol(2,0)<-35)
            {
                tol(2,0)=-35;
            }
        ROS_INFO("tol1 =%lf tol2 = %lf tol3=%lf ",tol(0,0),tol(1,0),tol(2,0));
        can_msgs::Frame frames;
        send_message = joint1.MomentOutput(tol(0,0));
        socketcan_send_pub.publish(send_message);

        send_message = joint2.MomentOutput(tol(1,0));
        socketcan_send_pub.publish(send_message);

        send_message = joint3.MomentOutput80(tol(2,0));
        socketcan_send_pub.publish(send_message);

        // 关节4位置控制 joint4angle其实是期望角度
        if(joint4angle<-M_PI)
        {
            joint4angle=-M_PI;
        }
        if(joint4angle>M_PI)
        {
            joint4angle=M_PI;     
        }
        //send_message = joint4.set_angle_for_new_joint(joint4angle);
        //socketcan_send_pub.publish(send_message);
        //send_message =joint4.set_angle(joint4angle);
        //socketcan_send_pub.publish(send_message);
         //ROS_INFO("joint4angle=%lf",joint4angle);

        // 关节5力矩控制
        if(KB_D>0.01)
        {
           //tol5 = 30;
            tol5 = 20;
            if(joint5_actual_angle>5.5)
            {
                tol5 = 0;
            }
        }
        else if (KB_D<-0.01)
        {
           //tol5 = -45;
            tol5 = -35;
            if(joint5_actual_angle<0.00001)
            {
                tol5 = 0;
             }
        }
        else
        {
            tol5 = 0;
        }
        //send_message = joint5.MomentOutput(tol5);
        //socketcan_send_pub.publish(send_message);

        frames.id = 0x80;
        frames.dlc = 0;
        socketcan_send_pub.publish(frames);
    }
}

//s 求向量符号函数
Vector3d sgns(Vector3d& s)
{
    Vector3d tempp;
    for(int i=0;i<3;i++)
    {
        if (s(i,0)>0)
        {
            tempp(i,0)=1;
            }
        else if(s(i,0)<0)
        {
            tempp(i,0)=-1;
            }
        else
        {
            tempp(i,0)=0;
        }
    }
    return tempp;

}

// 遥操作回调函数
void TeleOperationCallback(const  sensor_msgs::Imu& msg){
    expect_q(0,0) = msg.orientation.x;
    expect_q(1,0) = msg.orientation.y;
    expect_q(2,0) = msg.orientation.z;
    KB_D=msg.orientation.w;

    ROS_INFO("expect_q1=%lf",expect_q(0,0));
    ROS_INFO("expect_q2=%lf",expect_q(1,0));
    ROS_INFO("expect_q3=%lf",expect_q(2,0));
    ROS_INFO("KB_Ddddddd=%lf",KB_D);

    expect_dq(0,0) = msg.angular_velocity.x;
    expect_dq(1,0) = msg.angular_velocity.y;
    expect_dq(2,0) = msg.angular_velocity.z;

    expect_ddq(0,0) = msg.linear_acceleration.x;
    expect_ddq(1,0) = msg.linear_acceleration.y;
    expect_ddq(2,0) = msg.linear_acceleration.z;
}

void MainpulatorCallback(const can_msgs::Frame &receive_message)
{
        switch (receive_message.id)
        {
        case 0x181:
            joint1.current_angle(receive_message);
            q(0, 0) = joint1.get_current_angle();
            ROS_INFO("Joint1 = %lf", q(0, 0));
            break;
        case 0x182:
            joint2.current_angle(receive_message);
            q(1, 0) = joint2.get_current_angle();
            ROS_INFO("Joint1 = %lf", q(2, 0));
            break;
        case 0x183:
            joint3.current_angle(receive_message);
            q(2, 0) = joint3.get_current_angle();
            break;
        case 0x184:
            joint4.current_angle(receive_message);
            joint4_actual_angle = joint4.get_current_angle();
            break;
        case 0x185:
            joint5.current_angle(receive_message);
            joint5_actual_angle = joint5.get_current_angle();
            // ROS_INFO("joint5angle: %f, joint5_expect_angle: %f", joint5_actual_angle, joint5angle);
            break;

        case 0x281:
            joint1.current_velocity(receive_message);
            dq(0, 0) = joint1.get_current_velocity();
            break;
        case 0x282:
            joint2.current_velocity(receive_message);
            dq(1, 0) = joint2.get_current_velocity();
            break;
        case 0x283:
            joint3.current_velocity(receive_message);
            dq(2, 0) = joint3.get_current_velocity();
            break;
        case 0x284:
            joint4.current_velocity(receive_message);
            joint4_actual_velocity = joint4.get_current_velocity();
            break;
        case 0x285:
            joint5.ActualCurrent(receive_message);
            joint5_actual_current = joint5.get_ActualCurrent();
            break;

        default:
            break;
        }
    
}

void  AdaptiveBACKSTEPPINGparam_init()
{
        for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            lambda1(i,j) = 0;
            lambda2(i,j) = 0;
            kexi(i,j) = 0;
        }
    }
     //参数需进行调试
  //  lambda1(0,0)  = 15;
  //lambda1(1,1)  = 27;
   // lambda1(2,2) = 31; 
  //  lambda2(0,0)  = 21;
   // lambda2(1,1)  = 27;
   // lambda2(2,2) = 31; 
    // lambda1 can not be too small
    //lambda1(0,0)  = 5;
    //lambda1(1,1)  = 4;
    //lambda1(2,2)  = 4.6; 
    //lambda2(0,0)  = 27;
    //lambda2(1,1)  = 11;
    //lambda2(2,2)  = 31; 
    lambda1(0,0)  = 7.5;
    lambda1(1,1)  = 5;
    lambda1(2,2)  = 4; 
    lambda2(0,0)  = 120;
    lambda2(1,1)  = 120;
    lambda2(2,2)  = 92; 

    // for(int i=0;i<11;i++)
    //{
     //   theta_d(i,0) = 0;
   // }

    // 一开始ok的我的参数
    /*
    theta(0,0) = 13.5;//24;//8;
    theta(1,0) =2.5;//6.5;// 5;
    theta(2,0) =0;//-7;
    theta(3,0) =0;//-2;// -7;
    theta(4,0)  =0;// -7;
    theta(5,0) =0;//9.2;
    theta(6,0) =0;// 15;//15;//12;
    theta(7,0) = 0;//12.5;
    theta(8,0) = 0;//(-15~15)
    theta(9,0) = 0;//(-80~-30)
    theta(10,0) = 0;//(-5~20)
        
    theta_min(0,0) = 1;
    theta_max(0,0) = 16;
    theta_min(1,0) = 0.1;
    theta_max(1,0) = 10;
    theta_min(2,0) = -100;//-10;
    theta_max(2,0) = 100;//10;
    theta_max(3,0) = 50;//10;
    theta_min(3,0)  = -50;//-2;
    theta_max(4,0) = 30;//3;
    theta_min(4,0)  =-300;//-10;
    theta_max(5,0) = 100;//10;
    theta_min(5,0)  = -20;//-2;
    theta_max(6,0) = 50;//20;
    theta_min(6,0)  = -10;//13.5;
    theta_max(7,0) =20;// 15;
    theta_min(6,0)  = 0;//0;
    theta_max(8,0) = 100;//50;
    theta_min(8,0)  =-100;// -50;
    theta_max(9,0) = 50;//50;
    theta_min(9,0)  =-50;// -50;
    theta_max(10,0) = 50;//50;
    theta_min(10,0)  =-50;// -50;
*/
    // 娜姐实验的参数
    theta(0,0) = 8;
    theta(1,0) = 4;
    theta(2,0) = 4.5;
    theta(3,0) = 4.5;
    theta(4,0) = 1.5;
    theta(5,0) = 4;
    theta(6,0) = 3;
    theta(7,0) = 3.5;
    theta(8,0) = 0;
    theta(9,0) = 0;
    theta(10,0) = 0; 
    theta_min(0,0) = 6;
    theta_max(0,0) = 10;
    theta_min(1,0) = 2;
    theta_max(1,0) = 6;
    theta_min(2,0) = 4;
    theta_max(2,0) = 5;
    theta_min(3,0) = 4;
    theta_max(3,0) = 5;
    theta_min(4,0) = 1;   
    theta_max(4,0) = 2;
    theta_min(5,0) = 4;
    theta_max(5,0) = 16;
    theta_min(6,0) = 2;
    theta_max(6,0) = 15;
    theta_min(7,0) = 3;
    theta_max(7,0) = 12;
    theta_min(8,0) = -30;
    theta_max(8,0) = 30;
    theta_min(9,0) = -50;
    theta_max(9,0) = 50;
    theta_min(10,0) = -30;
    theta_max(10,0) = 30;
    // 一开始ok的我的自适应参数
    //gammamamm(0,0) = 10;//10;//650;//650;
    //gammamamm(1,1) = 10;//10;//250;//250;
    //gammamamm(2,2) = 0;//1000;//1000;
    //gammamamm(3,3) = 0;//1000;//1000;
    //gammamamm(4,4) = 0;//1000;//1000;
    //gammamamm(5,5) = 0;//1000;//1000;
    //gammamamm(6,6) = 0;//1000;//1000;
    //gammamamm(7,7) = 0;//1000;// 1000;
    //gammamamm(8,8) = 2000;//6000;//6000
    //gammamamm(9,9) = 1500;//2000;//2000
    //gammamamm(10,10) = 2000;//2000;//3000
    // 娜姐实验的参数
    gammamamm(0,0) = 500;
    gammamamm(1,1) = 100;

    gammamamm(2,2) = 1000;
    gammamamm(3,3) = 1200;
    gammamamm(4,4) = 1500;

    gammamamm(5,5) = 1500;
    gammamamm(6,6) = 1500;
    gammamamm(7,7) = 1500;

    gammamamm(8,8) = 1000;
    gammamamm(9,9) = 1000;
    gammamamm(10,10) = 1000;
}

void  ForceObserverparam_init()
{
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            L(i,j) = 0;  //NDOB参数
        }
    }
    //NDOB参数
    //L(0,0)  = 15.6/20;
    //L(1,1)  = 9.7/20;
    //L(2,2) = 15.7/20; 
    L(0,0)  = 15.6;
    L(1,1)  = 9.7;
    L(2,2) = 15.7; 
}
//反步控制，返回tol
Vector3d AdaptiveBackstepping(Vector3d& expect_q,Vector3d& expect_dq,Vector3d& expect_ddq, Vector3d& q, Vector3d& dq, Vector3d& tol_e_hat, double t)
{
    // 控制器设计
    z1 = q-expect_q;
    dz1 = dq-expect_dq;
    alpha1 = -lambda1 * z1 + expect_dq;
    dalpha1 = -lambda1 * dz1 + expect_ddq;
    z2 = dq - alpha1;
   //r=dz1+kexi*z1;
    //Vector3d qr_dot = expect_dq;//- k2*0;
    //Vector3d dqr_dot = expect_ddq;// - k2*0;

    // 令M*dalpha1+C*alpha1+G+F = fai^T * theta
    // theta = [m1 m2 fv1 fv2 fv3 fc1 fc2 fc3 d1 d2 d3];
     // m1 m2
    fai(0,0) = -1.0*( 1.0 / 3.0 * l1 * l1 * cos(q(1,0)) * cos(q(1,0)) * dalpha1(0,0) - 
                      2.0 / 3.0 * l1 * l1 * cos(q(1,0)) * sin(q(1,0)) * alpha1(0,0) );
    fai(0,1) =  -1.0*( (l1 * l1 * cos(q(1,0)) * cos(q(1,0)) +  l1 * l2 * cos(q(1,0)) * cos(q(1,0) - q(2,0)) + 1.0/ 3.0 * l2 * l2 * cos(q(1,0)-q(2,0)) *cos(q(1,0)-q(2,0)) ) * dalpha1(0,0) -
                       (2.0 * l1 * l1 * cos(q(1,0)) * sin(q(1,0)) + l1 * l2 * sin(2*q(1,0)-q(2,0)) + 2.0 / 3.0 * l2 * l2 * cos(q(1,0) - q(2,0)) * sin(q(1,0) - q(2,0)) ) * dq(1,0) * alpha1(0,0) +
                       ( l1 * l2 * cos(q(1,0)) * sin(q(1,0) - q(2,0)) + 2.0 / 3.0 * l2 * l2 * sin(q(1,0) - q(2,0)) * cos(q(1,0) - q(2,0))) * dq(2,0) * alpha1(0,0) );
    // 粘滞
    fai(0,2) = -dq(0,0);
    fai(0,3) = 0;
    fai(0,4) = 0;
    //库伦
    fai(0,5) = -atan2(900*dq(0,0),1)*2.0/(M_PI*1.0);
    fai(0,6) = 0;
    fai(0,7) = 0;
    //d
    fai(0,8) = 1;    
    fai(0,9) = 0;
    fai(0,10) = 0;

    fai(1,0) = -1.0*( (1.0 / 3.0 * l1 * l1 * dalpha1(1,0)) + 
                       1.0 / 2.0 * g * l1 * cos(q(1,0)) + 
                       1.0 / 3.0 * l1 * l1 * cos(q(1,0)) * sin(q(1,0)) * dq(0,0) * alpha1(0,0));
    fai(1,1) = -1.0*( (1.0 / 3.0 * l2 * l2 + l1 * l1 + l1 * l2 * cos(q(2,0))) * dalpha1(1,0) -  
                        (1.0 / 3.0 * l2 * l2 + 1.0 / 2.0 * l1 * l2 * cos(q(2,0))) * dalpha1(2,0) + 
                        (l1 * l1 * cos(q(1,0)) * sin(q(1,0)) + 1.0 / 2.0 * l1 * l2 * sin(2*q(1,0)-q(2,0)) + 1.0 / 3.0* l2 * l2 * cos(q(1,0)-q(2,0)) * sin(q(1,0)-q(2,0))) * dq(0,0) * alpha1(0,0) -
                        l1 * l2 * sin(q(2,0)) * dq(2,0) * alpha1(1,0) +
                        1.0 / 2.0 * l1 * l2 * sin(q(2,0)) * dq(2,0) * alpha1(2,0) +   
                        g * l1 * cos(q(1,0))  + 1.0 / 2.0 * g * l2 * cos(q(1,0) - q(2,0)) );
    // 粘滞
    fai(1,2) = 0;
    fai(1,3) = -dq(1,0);
    fai(1,4) = 0;
    //库伦
    fai(1,5) = 0;
    fai(1,6) = -atan2(900*dq(1,0),1)*2.0/(M_PI*1.0);
    fai(1,7) = 0;
    //d
    fai(1,8) = 0;    
    fai(1,9) = 1;
    fai(1,10) = 0;
    
    fai(2,0) = 0;
    fai(2,1) = -1.0*( -(1.0 / 3.0 * l2 * l2 + 1.0 / 2.0 * l1 * l2 * cos(q((2,0)))) * dalpha1(1,0) +
                       1.0 / 3.0 * l2 * l2 * dalpha1(2,0) - 
                       (1.0 / 3.0 * l2 * l2 * cos(q(1,0)-q(2,0)) * sin(q(1,0)-q(2,0)) + 1.0 / 2.0 * l1 * l2 * cos(q(1,0)) * sin(q(1,0)-q(2,0)) )* dq(0,0) * alpha1(0,0) +
                       1.0 / 2.0 * l1 * l2 * sin(q(2,0)) * dq(1,0) * alpha1(1,0) - 
                       1.0 / 2.0 * g * l2 * cos(q(1,0)-q(2,0)) );
    // 粘滞
    fai(2,2) = 0;
    fai(2,3) = 0;
    fai(2,4) = -dq(2,0);
    //库伦
    fai(2,5) = 0;
    fai(2,6) = 0;
    fai(2,7) = -atan2(900*dq(2,0),1)*2.0/(M_PI*1.0);
    //d
    fai(2,8) = 0;    
    fai(2,9) = 0;
    fai(2,10) = 1.0;

theta_d = gammamamm*fai.transpose()*z2*t;

for(int i=0;i<11;i++)
    {
        if(theta(i,0)>=theta_max(i,0)&&theta_d(i,0)>0)
        {
            theta_d(i,0) = 0;
        }
        if(theta(i,0)<=theta_min(i,0)&&theta_d(i,0)<0)
        {
            theta_d(i,0) = 0;
        }
    }
  theta = theta +  theta_d;
 //Vector3d u = Mn * dalpha1 + Cn *dq + Gn- Mn * (z1 + lambda2 * z2) - tol_e_hat;
 //Vector3d u = Mn * dalpha1 + Cn *dq + Gn- Mn * (z1 + lambda2 * z2) ;
   //Vector3d u = Mn * dalpha1 + Cn *dq+ Gn  + fx - Mn * (z1 + lambda2 * z2)- tol_e_hat-5*r;
   Vector3d ua = -1.0*fai*theta;
   Vector3d us = - z1 - lambda2*z2;
   //ROS_INFO("ua1 =%lf ua2 = %lf ua3=%lf ",ua(0,0),ua(1,0),ua(2,0));
   //ROS_INFO("us1 =%lf us2 = %lf us3=%lf ",us(0,0),us(1,0),us(2,0));
   
   //u=-1.0*fai*theta - z1 - lambda2*z2;
  // u=-1.0*fai_dot*theta - z1 - lambda2*z2;
    //u(1,0)=-u(1,0);
  // u(1,0)=0;
   // Vector3d u = Mn * dalpha1 + Cn *dq+ Gn  + fx - Mn * (z1 + lambda2 * z2);
    // 调试用：
    // u = ua;  //先检查模型补偿项，应该占很大的部分,
    // u = us;  //再检查反馈项，应该占很小的部分
    u = ua + us;
    return u;  //输出tol_s
}


//  非线性扰动观测器，返回d_hat
Vector3d NDOB(Vector3d& tol,Vector3d& q,Vector3d& dq,double t)
{
     // Mn矩阵
    Matrix3d Mn_templete;
    Mn_templete(0,0) = 1.0 / 3.0 * m1 * l1 * l1 * cos(q(1,0)) * cos(q(1,0)) 
                                 + m2 *( l1 * l1 * cos(q(1,0)) * cos(q(1,0)) + 
                                         l1 * l2 * cos(q(1,0)) * cos(q(1,0)-q(2,0)) + 
                                         1.0 / 3.0 * l2 * l2 * cos(q(1,0)-q(2,0)) * cos(q(1,0)-q(2,0)));
    Mn_templete(0,1) = 0;
    Mn_templete(0,2) = 0;

    Mn_templete(1,0) = 0;
    Mn_templete(1,1) = 1.0 / 3.0 * m1 * l1 *l1 + m2 * l1 *l1 + 1.0 / 3.0 * m2 * l2 *l2 + m2 * l1 * l2 * cos(q(2,0));
    Mn_templete(1,2) = - 1.0 / 3.0 * m2 * l2 *l2 - 1.0 / 2.0 * m2 * l1 * l2 * cos(q(2,0));  

    Mn_templete(2,0) = 0;
    Mn_templete(2,1) = - 1.0 / 3.0 * m2 * l2 *l2 - 1.0 / 2.0 * m2 * l1 * l2 * cos(q(2,0));  
    Mn_templete(2,2) = 1.0 / 3.0 * m2 * l2 *l2; 

    // Cn矩阵
    Matrix3d Cn_templete;
    Cn_templete(0,0) = m2 * (l1 * l2 * cos(q(1,0)) * sin(q(1,0)-q(2,0)) + 
                             2.0 / 3.0 * l2 * l2 * cos(q(1,0)-q(2,0)) * sin(q(1,0)-q(2,0))) * dq(2,0) - 
                       ( 2.0 / 3.0 * m1 * l1 * l1 * cos(q(1,0)) * sin(q(1,0)) + 
                       m2 * (2 * l1 * l1 * cos(q(1,0)) * sin(q(1,0)) + l1 * l2 * sin(2 * q(1,0)-q(2,0)) + 
                             2.0 / 3.0 * l2 * l2 * cos(q(1,0)-q(2,0)) * sin(q(1,0)-q(2,0))))*(dq(1,0));
    Cn_templete(0,1) = 0;
    Cn_templete(0,2) = 0;

    Cn_templete(1,0) = (1.0 / 3.0 * m1 * l1 *l1 * cos(q(1,0)) * sin(q(1,0)) + 
                       m2 * (l1 * l1 * cos(q(1,0)) * sin(q(1,0)) + 
                            1.0 / 2.0 * l1 * l2 * sin(2*q(1,0)-q(2,0))) + 
                            1.0 /3.0 * l2 * l2 * cos(q(1,0)-q(2,0)) * sin(q(1,0)-q(2,0)))*dq(0,0);
    Cn_templete(1,1)= -m2 * l1 * l2 * sin(q(2,0)) * dq(2,0);
    Cn_templete(1,2)= 1.0 / 2.0 * m2 * l1 * l2 * sin(q(2,0)) * dq(2,0);

    Cn_templete(2,0)= -m2 * (1.0 / 2.0 * l1 * l2 * cos(q(1,0)) * sin(q(1,0)-q(2,0)) + 
                             1.0 / 3.0 * l2 * l2 * cos(q(1,0)-q(2,0)) * sin(q(1,0)-q(2,0))) * dq(0,0);
    Cn_templete(2,1)= 1.0 / 2.0 * m2 * l1 * l2 * sin(q(2,0)) * dq(1,0);
    Cn_templete(2,2)=0;
    // Gn矩阵
    Vector3d Gn_templete;
    Gn_templete(0,0)=0;
    Gn_templete(1,0)=(1.0/2.0 * m1 + m2) * g * l1 * cos(q(1,0)) + 1.0/2.0 * m2 * g * l2 * cos(q(1,0)-q(2,0));
    Gn_templete(2,0)= -1.0/2.0 * m2 * g * l2 * cos(q(1,0)-q(2,0));
    // Fn 矩阵
    Vector3d Fn_templete;
    Fn_templete(0,0) = fv(0,0) * dq(0,0) + fc(0,0) * atan2(900*dq(0,0),1)*2.0/(M_PI*1.0);
    Fn_templete(1,0) = fv(1,0) * dq(1,0) + fc(1,0) * atan2(900*dq(1,0),1)*2.0/(M_PI*1.0);
    Fn_templete(2,0) = fv(2,0) * dq(2,0) + fc(2,0) * atan2(900*dq(2,0),1)*2.0/(M_PI*1.0);
    
    double k = 1.0;
    xi_dot = -L*xi-L*(-Cn_templete*dq-Gn_templete - Fn_templete + k*tol + L*Mn_templete*dq);  //rignt
    xi = xi + xi_dot * t;    
    d_hat = xi + L * Mn_templete * dq;

    //ROS_INFO("fx1=%f fx2=%f fx3=%f",Fn_templete(0,0),Fn_templete(1,0),Fn_templete(2,0));
    ROS_INFO("G1=%f G2=%f G3=%f",Gn_templete(0,0),Gn_templete(1,0),Gn_templete(2,0));
    ROS_INFO("d_hat1=%f d_hat2=%f d_hat3=%f",d_hat(0,0),d_hat(1,0),d_hat(2,0));
    return  d_hat;



    //xi_dot = -L*xi-L*(-Cn_templete*dq - fx +k*tol+L*Mn_templete*dq);   // without Gn
    //xi = xi + xi_dot * t;
    //d_hat = xi + L * Mn_templete * dq  - Gn_templete/2 ;


    //Gn(1,0)=-Gn(1,0);
    //xi_dot = -L*xi-L*(-Cn*dq-Gn+k*tol+L*Mn*dq);+
    //xi_dot = -L*xi-L*(-Cn*dq-Gn - fx+k*tol+L*Mn*dq);  //rignt
   //Vector3d temp_tol = tol;
   //temp_tol(1,0) = -temp_tol(1,0);
    //xi_dot = -L*xi-L*(-Cn_templete*dq - fx +k*tol+L*Mn_templete*dq);   // without Gn
    //xi_dot = -L*xi-L*(-Cn*dq + fx+L*Mn*dq);   // without Gn and tol  (-10,10)N fangbo
    //ROS_INFO("fx1=%f fx2=%f fx3=%f",fx(0,0),fx(1,0),fx(2,0));
    //ROS_INFO("G1=%f G2=%f G3=%f",Gn(0,0),Gn(1,0),Gn(2,0));
    // 更新xi_迭代周期不知道是多少
    //xi = xi + xi_dot * t;
    //ROS_INFO("xi1=%f xi2=%f xi3=%f",xi(0,0),xi(1,0),xi(2,0));
    //d_hat = xi + L * Mn_templete * dq  - Gn_templete/2 ;
    //ROS_INFO("d_hat1=%f d_hat2=%f d_hat3=%f",d_hat(0,0),d_hat(1,0),d_hat(2,0));
    //u = d_hat;
    
}