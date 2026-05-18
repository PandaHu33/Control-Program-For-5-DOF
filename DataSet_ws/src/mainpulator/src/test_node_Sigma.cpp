// ReadMe：
// 此程序适用于主端设备为Sigma7力反馈手，并搭配基于Unity的UI界面使用
// 此程序接收来自Sigma7的末端位置和姿态，进行位置映射、逆运动学求解、约束规划并进行机械臂自适应控制
// 此程序使用多种方法进行机械臂外力估计，并换算成末端力发送给Sigma7力反馈手
// Sigma7力反馈手运行程序名【Sigma7】,请先运行此程序，在运行Sigma7的程序
// 基于Unity的UI界面控制机械臂的【复位】与【开启运动】，使用完程序，请务必通过UI界面将机械臂【复位】（伸直状态）
// 与Unity通信依赖ROS-TCP-Endpoint包，请确保此程序包正确下载到src文件夹下，并在程序运行时开启endpoint.launch结点
// 运行前先将文件名改为test_node.cpp再编译运行
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
#include <std_msgs/Float32.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <mainpulator/PipeFilterExpansion.h>   //包含轨迹规划的头文件
#include <qpOASES.hpp>  //求解QP问题的库，类似于Matlab的quadprog

#define PI acos(-1)
using namespace std;
using namespace Eigen;
using namespace qpOASES;
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
//double m1 = 9.6;
//double m2  = 6.4;
//double m2  = 1.5;  //陆上参数
double l1 = 0.424;
double l2 = 0.268;
// 水里的参数
double m1 = 8;
double m2  = 0.5;
//Matrix3Xd Mn(3,3);
//Matrix3Xd Cn(3,3);
//Vector3d Gn;

//g根据安装方向修改 橙色底座为-------
//double g = 9.8;
double g = 7.8;  // 水中g变小
//实际角度
Vector3d q;
//实际角速度
Vector3d dq;
//期望角度
Vector3d expect_q;
//期望角速度
Vector3d expect_dq;
//期望角加速度
Vector3d expect_ddq;

double expect_q4 =0.0001;  //关节4期望角度

Vector3d tol;
Matrix3d Jaco;

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
MatrixXd fai(3,11);

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
Vector3d d_NDOB_hat;
double L_NDOB; // 测试一下自适应变化增益

// LDOB参数和相关变量
Vector3d ddq;  //微分得到的角加速度
Vector3d pre_dq;  //上周期加速度
Vector3d d_LDOB_hat;
Vector3d pre_d_LDOB_hat;
double cutoff_freq; //截止频率

//LESO参数和相关变量定义
Matrix3d x_LESO_hat;
Vector3d d_LESO_hat;
Matrix3d pre_x_LESO_hat;
double bandwidth;  //观测器带宽，建议3-5倍控制系统带宽

// NESO参数和相关变量定义
Matrix3d x_NESO_hat;
Vector3d d_NESO_hat;
Matrix3d pre_x_NESO_hat;
double beta1_;
double beta2_;
double beta3_;
double alpha1_;
double alpha2_;
double delta_;

// FGM参数和相关变量定义
Vector3d d_FGM_hat;
Vector3d pre_integrand;
Vector3d integrand;
Matrix3d K_gain;
Vector3d p;
Vector3d p_hat;

// AFO参数和相关变量定义
Vector3d d_AFO_hat;
bool is_first_sample; //是不是第一次采样标志位
Vector3d x2_AFO_star; // x2滤波后
Vector3d Omega_AFO_star; // Omega滤波后
Matrix3d sigma_AFO_star; // sigma滤波后
//double w_AFO;  //截止频率
double a_AFO; // 中间自适应律参数
Matrix3d Gamma1_AFO;  //中间自适应律
Vector3d Gamma2_AFO; 
Matrix3d kappa_AFO; // 增益学习律
double lambda_AFO; // 外力估计律参数1
double eplsion_AFO; // 外力估计律参数2

// 外力相关的变量
Vector3d Fe; //末端力
Vector3d tol_e;  // 关节外力矩
Vector3d tol_e_hat;
Vector3d tol_zk; //估计外力补偿后的输入力矩

Vector3d zerovector;

// 主从遥操作相关参数和变量------------------------------------------------------------------------------------------
// 若主端为Omega7等并联设备，则接收到来自主端的末端位置，需在从端完成异构映射、电动机械臂的逆运动学求解和轨迹规划
// 定义主端Omega7传过来的末端位置，数据类型：geometry_msgs::Vector3Stamped
Vector3d Pm;  //Omega7传输过来的末端位置
Vector3d Psd; //机械臂期望的末端位置（映射后的）
Vector3d Ps;  //机械臂实际末端位置
Vector3d Drm; //主端末端姿态
Vector3d expect_q_inv; //经逆运动学求解后的关节
Matrix3d expect_q_traj; // 经关节轨迹规划后的期望角度、角速度和角加速度矩阵
// 定义异构映射的参数
//Vector3d Kp;
//Vector3d Tp;
// 定义轨迹规划相关的参数
double t_last=0.0;
double t = 0.0;
double q1_res_data;
double q2_res_data;
double q3_res_data;
double q1_result_last;
double q2_result_last;
double q3_result_last;
double q1_velocity;
double q2_velocity;
double q3_velocity;
double q1_velocity_last;
double q2_velocity_last;
double q3_velocity_last;
double q1_acceleration;
double q2_acceleration;
double q3_acceleration;
double compensate_q00=0.0;
double compensate_q10=0.0;
double compensate_q20=0.0;
//定义与omega通信测试时的变量
double force_count;
double grip_count;
bool receive_omega_flag = false;

//定义与Unity的指令变量
//double GripperFlag = 0.0;  // 抓手运动指令： 0不动，-1关闭，1打开
double ForceFeedbackFlag = 0.0;  // 力反馈开启指令：0 OFF，1 ON
double MappingFlag = 0.0; //映射模式指令：0位置映射，1增量映射（先保持为位置映射）
double MovementFalg = 0.0; //归位和开始运动指令：0归位为初始位置，1开始运动

// 定义基于JLCT约束规划器的变量和参数-对称约束
const Vector3d Angle_LIMITS(M_PI/2, M_PI/2, M_PI);  //关节角度约束
const Vector3d AngleVelocity_LIMITS(1.0, 1.0, 1.0); //关节角速度
const Vector3d AngleAcceleration_LIMITS(3.0, 3.0, 3.0); //关节角加速度约束
const Vector3d Jerk_LIMITS(12, 12, 12); //关节Jerk约束
const double LAMBDA_P = 5.0;
const double LAMBDA_V = 5.0;
const double LAMBDA_A = 5.0;
const double XI = 10.0;

struct TrajectoryState{    //定义储存轨迹规划状态的结构体
    Vector3d joint_jerk;
    Vector3d joint_a;
    Vector3d joint_v;
    Vector3d joint_p;
};

//--------------------------------------------------------------------------------------------------------------------

void  AdaptiveBACKSTEPPINGparam_init();
void  ForceObserverparam_init();

// 控制器
Vector3d AdaptiveBackstepping(Vector3d& expect_q, Vector3d& expect_dq, Vector3d& expect_ddq, Vector3d& q, Vector3d& dq, Vector3d& tol_e_hat, double t);

// 观测器
Vector3d LDOB(Vector3d& tol,Vector3d& q,Vector3d& dq,Vector3d& pre_dq,Vector3d& pre_d_LDOB_hat,double t);//线性扰动观测器
Vector3d NDOB(Vector3d& tol,Vector3d& q,Vector3d& dq,double t); //非线性扰动观测器
Matrix3d LESO(Vector3d& tol,Vector3d& q,Vector3d& dq,Matrix3d& pre_x_LESO_hat,double t);// 线性扩展状态观测器
Matrix3d NESO(Vector3d& tol,Vector3d& q,Vector3d& dq,Matrix3d& pre_x_NESO_hat,double t);// 非线性扩展状态观测器
Vector3d fal(Vector3d& e_, double alpha_, double delta_); // NESO中使用的非线性误差函数
Vector3d FirstOrderGM(Vector3d& tol,Vector3d& q,Vector3d& dq, Vector3d& pre_integrand, double t); //一阶动量观测器
Vector3d AFO(Vector3d& tol,Vector3d& q,Vector3d& dq, Vector3d& pre_d_AFO_hat, double t); // 自适应力观测器

//矩阵计算
Matrix3d Mn(Vector3d& q);
Matrix3d Cn(Vector3d& q,Vector3d& dq);
Vector3d Gn(Vector3d& q);
Vector3d Fn(Vector3d& dq);
Matrix3d Jn(Vector3d& q);//求解机械臂的雅可比矩
Vector3d sgns(Vector3d& s);  // 符号函数

// 关节力矩换算成末端力
Vector3d ComputeEndForce(Vector3d& tol_e,Matrix3d& Jaco);

// 定义Omega7需要的异构映射等函数
Vector3d PositionMapping(Vector3d& Pm);  //位置映射,输入Pm,输出Psd
Vector3d ManipulatorInverseKinetic(Vector3d& Psd); //逆运动学求解，输入Psd，输出expect_q_inv
Matrix3d Joint_TrajectoryPlanning(Vector3d& expect_q_inv); //关节轨迹规划, 返回机械臂的期望角度、角速度和角加速度矩阵expect_q_traj
//约束规划器， 返回机械臂的期望角度、角速度和角加速度的轨迹状态结构体
TrajectoryState ConstraitTrajectoryPlanning(Vector3d& expect_q_inv, TrajectoryState& last_state, double t);

// 与Omega等主端通信的回调函数
void Omega7_PositionCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);  //收到Omega7的末端位置
void Omega7_DirectionCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);  //收到Omega7的末端姿态
void Omega7_GripCallback(const std_msgs::Float32::ConstPtr& msg);   //收到Omega7的抓手状态
void Omega7_ButtonsCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);  //收到Omega7的button


void MainpulatorCallback(const can_msgs::Frame &receive_message);
/** 
 * @brief 位置模式回调函数,编码器返回速度、位置等信息
 * @param receive_message ros_canopen类对象
 * @return 
 */

//void TeleOperationCallback(const  sensor_msgs::Imu& msg);

void UnityCommand_Callback(const  std_msgs::Float64MultiArray& msg);


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

    // 此订阅器和发布器针对主端Phantom Omni/Preium力反馈手，与Matlab通信
    // 订阅器TeleOperation_receive_sub订阅遥操作期望信息，话题为"/pub_joint_state"，回调函数为TeleOperationCallback
    ros::NodeHandle TeleOperation;
    //ros::Subscriber TeleOperation_receive_sub = TeleOperation.subscribe("/pub_joint_state",10, TeleOperationCallback);  
    // 发布信息给Matlab
    ros::Publisher TeleOperation_pub = TeleOperation.advertise<sensor_msgs::JointState>("/Matlab/dataplot",10);
    ros::Publisher TeleOperation_pub2 = TeleOperation.advertise<sensor_msgs::JointState>("/Matlab/dataplot2",10);
     
    /*以下是主端为Omega7的订阅与发布方式*/
    ros::NodeHandle OmegaNode;
    //订阅Omega7主端信息，分别订阅位置、位姿、抓手状态和按钮信息，对应四个订阅器和四个回调函数
    ros::Subscriber omega_position_sub_ = OmegaNode.subscribe("/Omega7/position", 10, Omega7_PositionCallback);
    ros::Subscriber omega_direction_sub_ = OmegaNode.subscribe("/Omega7/direction", 10, Omega7_DirectionCallback);
    ros::Subscriber omega_grip_sub_ = OmegaNode.subscribe("/Omega7/grip_state", 10, Omega7_GripCallback);
    ros::Subscriber omega_button_sub_ = OmegaNode.subscribe("/Omega7/button_state", 10, Omega7_ButtonsCallback);
    //发布给Omega7力信息，分别发布末端力和夹抓力，对应两个发布器
    ros::Publisher omega_force_pub_ = OmegaNode.advertise<geometry_msgs::Vector3>("/Slave/force", 10);
    ros::Publisher omega_grip_force_pub_ = OmegaNode.advertise<std_msgs::Float32>("/Slave/grip_force",10);

    
    //与Unity上位机界面通信
    ros::NodeHandle Unity_UI;
    ros::Publisher pub2UnityUI = Unity_UI.advertise<std_msgs::Float64MultiArray>("/UEM/Force", 10);  // 发送给Unity估计力
    ros::Subscriber subUnityCommands = Unity_UI.subscribe("/Unity/Commands", 10, UnityCommand_Callback); // 接收来自Unity的命令
 

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
    usleep(500000);
    //机械臂使能
    param::Enable(socket_can, 6, joint1);
    param::Enable(socket_can, 6, joint2);
    param::Enable(socket_can, 6, joint3);
    //param::Enable(socket_can, 6, joint4);
    param::Enable(socket_can, 6, joint5);
    usleep(500000);
    ROS_INFO("init end");

    AdaptiveBACKSTEPPINGparam_init();
    ForceObserverparam_init();

    // AFO相关变量初始化
    is_first_sample = true; //是不是第一次采样标志位
    kappa_AFO = Matrix3d::Zero(3,3);
    kappa_AFO.diagonal() << 0.1, 0.1, 0.1;
    L_NDOB = 5;

    // 摩擦参数初始化
    fc(0,0)  =  7.65;
    fc(1,0)  = 7.738;
    fc(2,0) =  6.319; 
    //fv(0,0)  =  19.6233;
    //fv(1,0)  = 13.758;
    //fv(2,0) = 12.865; 
    //水里的参数
    fv(0,0)  =  1.8212;
    fv(1,0)  = -4.7070;
    fv(2,0) = -3.0858; 
    
    // 约束规划状态初始化
    TrajectoryState current_traj_state;

    current_traj_state.joint_p = zerovector;
    current_traj_state.joint_v = zerovector;
    current_traj_state.joint_a = zerovector;
    current_traj_state.joint_jerk = zerovector;

    while (ros::ok)
    {
        static long run_times = 0;
        if (run_times++ % 100 == 0)
        {
            cout << "\033c";
        }
        // cout << "\033[2J\033[1;1H";
        cout << "\033[0;0H";
        //处理排队的回调函数
        ros::spinOnce();
        
        // 发布给主端Matlab角度、角速度、估计外力
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.position.resize(9);
        joint_state.velocity.resize(9);
        joint_state.effort.resize(18);

        joint_state.position[0] = expect_q_inv(0,0); // 逆运动学求解后的
        joint_state.position[1] = expect_q_inv(1,0);
        joint_state.position[2] = expect_q_inv(2,0);
        joint_state.position[3] = expect_q(0,0); // 轨迹规划后的期望关节角度
        joint_state.position[4] = expect_q(1,0);
        joint_state.position[5] = expect_q(2,0);
        joint_state.position[6] = q(0,0);  //机械臂实际关节角度
        joint_state.position[7] = q(1,0);
        joint_state.position[8] = q(2,0);

        joint_state.velocity[0] = expect_dq(0,0); // 轨迹规划后的期望关节角速度
        joint_state.velocity[1] = expect_dq(1,0);
        joint_state.velocity[2] = expect_dq(2,0);
        joint_state.velocity[3] = dq(0,0);  //机械臂实际关节角速度
        joint_state.velocity[4] = dq(1,0);  
        joint_state.velocity[5] = dq(2,0);
        joint_state.velocity[6] = expect_ddq(0,0);// 轨迹规划后的期望关节角加速度
        joint_state.velocity[7] = expect_ddq(1,0);  
        joint_state.velocity[8] = expect_ddq(2,0);

            
        //joint_state.effort[0] = tol_e_hat(0,0);  //估计的外力矩
        //joint_state.effort[1] = tol_e_hat(1,0);
        //joint_state.effort[2] = tol_e_hat(2,0);
        //joint_state.effort[3] = Fe(0,0); //估计的外力
        //joint_state.effort[4] = Fe(1,0);
        //joint_state.effort[5] = Fe(2,0);

        // 各观测器估计值-测试用
        joint_state.effort[0] = d_LDOB_hat(0,0);
        joint_state.effort[1] = d_LDOB_hat(1,0);
        joint_state.effort[2] = d_LDOB_hat(2,0);

        joint_state.effort[3] = d_NDOB_hat(0,0);
        joint_state.effort[4] = d_NDOB_hat(1,0);
        joint_state.effort[5] = d_NDOB_hat(2,0);

        joint_state.effort[6] = d_LESO_hat(0,0);
        joint_state.effort[7] = d_LESO_hat(1,0);
        joint_state.effort[8] = d_LESO_hat(2,0);

        //joint_state.effort[9] = d_NESO_hat(0,0);
        //joint_state.effort[10] = d_NESO_hat(1,0);
        //joint_state.effort[11] = d_NESO_hat(2,0);

        joint_state.effort[9] = d_FGM_hat(0,0);
        joint_state.effort[10] = d_FGM_hat(1,0);
        joint_state.effort[11] = d_FGM_hat(2,0);

        joint_state.effort[12] = d_AFO_hat(0,0);
        joint_state.effort[13] = d_AFO_hat(1,0);
        joint_state.effort[14] = d_AFO_hat(2,0);

        joint_state.effort[15] = Fe(0,0);
        joint_state.effort[16] = Fe(1,0);
        joint_state.effort[17] = Fe(2,0);


        TeleOperation_pub.publish(joint_state);

        // 发布给主端Matlab 其他数据
        sensor_msgs::JointState joint_state2;
        joint_state2.header.stamp = ros::Time::now();
        joint_state2.position.resize(9);
        joint_state2.velocity.resize(9);
        joint_state2.effort.resize(9);

        joint_state2.position[0] = tol(0,0); // 机械臂控制力矩
        joint_state2.position[1] = tol(1,0);
        joint_state2.position[2] = tol(2,0); 
        tol_zk = tol + d_AFO_hat;
        joint_state2.position[3] = tol_zk(0,0); // 外力估计补偿的力矩
        joint_state2.position[4] = tol_zk(1,0);
        joint_state2.position[5] = tol_zk(2,0);

        joint_state2.effort[0] = kappa_AFO(0,0);
        joint_state2.effort[1] = kappa_AFO(1,0);
        joint_state2.effort[2] = kappa_AFO(2,0);
        joint_state2.effort[3] = kappa_AFO(0,1);
        joint_state2.effort[4] = kappa_AFO(1,1);
        joint_state2.effort[5] = kappa_AFO(2,1);
        joint_state2.effort[6] = kappa_AFO(0,2);
        joint_state2.effort[7] = kappa_AFO(1,2);
        joint_state2.effort[8] = kappa_AFO(2,2);
        
        TeleOperation_pub2.publish(joint_state2);


        // 发送给Unity界面末端估计外力
        std_msgs::Float64MultiArray array_msg3;
        array_msg3.data.push_back(ForceFeedbackFlag*Fe(0,0));
        array_msg3.data.push_back(ForceFeedbackFlag*Fe(1,0));
        array_msg3.data.push_back(ForceFeedbackFlag*Fe(2,0));
        pub2UnityUI.publish(array_msg3);

        // 发布给主端Omega末端力-测试
        geometry_msgs::Vector3 force_msg;  //末端力
        //geometry_msgs::Vector3Stamped force_msg;  //末端力
        //force_msg.x = 0.0*ForceFeedbackFlag*Fe(0,0);//0.1*Fe(0,0);
        //force_msg.y = 0.0*ForceFeedbackFlag*Fe(1,0);//0.1*Fe(1,0);
        //force_msg.z = 0.0*ForceFeedbackFlag*Fe(2,0);//0.1*Fe(2,0);
        force_msg.x = 0.0;
        force_msg.y = 0.0;
        force_msg.z = 0.0;
        omega_force_pub_.publish(force_msg);
        ROS_INFO("Send force: x=%f, y =%f, z=%f", force_msg.x, force_msg.y, force_msg.z);

        //force_msg.x = 1.0*force_count;
        //force_msg.y = force_count;
        //force_msg.z = 0.0*force_count;
        //omega_force_pub_.publish(force_msg);
        //force_count=force_count+0.1f;
        //if(force_count>10) force_count=0;
        //ROS_INFO("force_count: %f", force_count);
        //ROS_INFO("Send force: x=%f, y =%f, z=%f", force_msg.x, force_msg.y, force_msg.z);

        // 当主手是Sigma7时，打开以下注释
        Psd = PositionMapping(Pm);  // 主从位置映射
        //ROS_INFO("Map_Psd: x=%f, y =%f, z=%f", Psd(0,0), Psd(1,0), Psd(2,0));
        
        if(MovementFalg > 0)
        {
            if(receive_omega_flag == true)
            {
            expect_q_inv = ManipulatorInverseKinetic(Psd); //逆运动学求解
            }
            else
            {
            expect_q_inv(0,0) = 0.00001;
            expect_q_inv(1,0) = 0.00001;
            expect_q_inv(2,0) = 0.00001;
            }
        }
        else
        {
            expect_q_inv(0,0) = 0.00001;
            expect_q_inv(1,0) = 0.00001;
            expect_q_inv(2,0) = 0.00001;
        }
        
       // ROS_INFO("inv_q: q1=%f, q2=%f, q3=%f", expect_q_inv(0,0), expect_q_inv(1,0), expect_q_inv(2,0));
    
        // 以下是通过约束规划器规划的期望轨迹（满足约束）
        current_traj_state = ConstraitTrajectoryPlanning(expect_q_inv, current_traj_state,0.02);
        expect_q = current_traj_state.joint_p;
        expect_dq = current_traj_state.joint_v;
        expect_ddq = current_traj_state.joint_a;
        //ROS_INFO("expect_q: q1=%f, q2=%f, q3=%f", expect_q(0,0), expect_q(1,0), expect_q(2,0));
        //ROS_INFO("expect_dq: dq1=%f, dq2=%f, dq3=%f", expect_dq(0,0), expect_dq(1,0), expect_dq(2,0));
        //ROS_INFO("expect_ddq: ddq1=%f, ddq2=%f, ddq3=%f", expect_dq(0,0), expect_dq(1,0), expect_dq(2,0));
        

        // 自适应反步控制
        tol=AdaptiveBackstepping(expect_q,expect_dq,expect_ddq,q,dq,zerovector,0.002);
        
        d_NDOB_hat = NDOB(tol,q,dq,0.01);
        ROS_INFO("d_NDOB_hat1 = %lf d_NDOB_hat2 = %lf d_NDOB_hat3 = %lf ",d_NDOB_hat(0,0),d_NDOB_hat(1,0),d_NDOB_hat(2,0));
        //ROS_INFO("L_NDOB = %lf",L_NDOB);
 
        d_LDOB_hat = LDOB(tol,q,dq,pre_dq,pre_d_LDOB_hat,0.01);
        ROS_INFO("d_LDOB_hat1 = %lf d_LDOB_hat2 = %lf d_LDOB_hat3 = %lf ",d_LDOB_hat(0,0),d_LDOB_hat(1,0),d_LDOB_hat(2,0));

        x_LESO_hat = LESO(tol,q,dq,pre_x_LESO_hat,0.01);
        d_LESO_hat = x_LESO_hat.col(2);
        ROS_INFO("d_LESO_hat1 = %lf d_LESO_hat2 = %lf d_LESO_hat3 = %lf ",d_LESO_hat(0,0),d_LESO_hat(1,0),d_LESO_hat(2,0));

        //x_NESO_hat = NESO(tol,q,dq,pre_x_NESO_hat,0.01);
        //d_NESO_hat = x_NESO_hat.col(2);
        //ROS_INFO("d_NESO_hat1 = %lf d_NESO_hat2 = %lf d_NESO_hat3 = %lf ",d_NESO_hat(0,0),d_NESO_hat(1,0),d_NESO_hat(2,0));

        d_FGM_hat = FirstOrderGM(tol,q,dq,pre_integrand,0.01);
        ROS_INFO("d_FGM_hat1 = %lf d_FGM_hat2 = %lf d_FGM_hat3 = %lf ",d_FGM_hat(0,0),d_FGM_hat(1,0),d_FGM_hat(2,0));
        
        d_AFO_hat = AFO(tol,q,dq,d_AFO_hat,0.01);
        ROS_INFO("d_AFO_hat1 = %lf d_AFO_hat2 = %lf d_AFO_hat3 = %lf ",d_AFO_hat(0,0),d_AFO_hat(1,0),d_AFO_hat(2,0));
        //ROS_INFO("x2_AFO_star = %lf, %lf, %lf",x2_AFO_star(0,0), x2_AFO_star(1,0), x2_AFO_star(2,0));
        //ROS_INFO("Omega_AFO_star = %lf, %lf, %lf", Omega_AFO_star(0,0),Omega_AFO_star(1,0),Omega_AFO_star(2,0));
        //ROS_INFO("sigma_AFO_star = %lf, %lf, %lf", sigma_AFO_star(0,0),sigma_AFO_star(1,1),sigma_AFO_star(2,2)); 
        //ROS_INFO("kappa_AFO = %lf, %lf, %lf",kappa_AFO(0,0),kappa_AFO(1,1),kappa_AFO(2,2));
        //ROS_INFO("Gamma1_AFO = %lf, %lf, %lf",Gamma1_AFO(0,0),Gamma1_AFO(1,1),Gamma1_AFO(2,2));

        
        //ROS_INFO("q = %lf, %lf, %lf", q(0,0), q(1,0), q(2,0));
       
        // 求末端力
        Jaco = Jn(q);
       // ROS_INFO("J11 = %lf J12 = %lf J13 = %lf; ", Jaco(0,0),Jaco(0,1), Jaco(0,2));
       // ROS_INFO("J21 = %lf J22 = %lf J23 = %lf; ", Jaco(1,0),Jaco(1,1), Jaco(1,2));
       // ROS_INFO("J31 = %lf J32 = %lf J33 = %lf; ", Jaco(2,0),Jaco(2,1), Jaco(2,2));
        //tol_e_hat = d_NDOB_hat;
        //Fe = ComputeEndForce(d_NDOB_hat, Jaco);
        Fe = MovementFalg*ComputeEndForce(d_AFO_hat, Jaco);//设置机械臂归位后力反馈为0
        ROS_INFO("Fx = %lf Fy = %lf Fz = %lf ",Fe(0,0),Fe(1,0),Fe(2,0));
        
       // ROS_INFO("theta_m1 = %lf theta_m2 = %lf",theta(0,0),theta(1,0));
       // ROS_INFO("theta_fv1 = %lf theta_fv2 = %lf, theta_fv3=%lf",theta(2,0),theta(3,0),theta(4,0));
       // ROS_INFO("theta_fc1 = %lf theta_fc2 = %lf, theta_fc3=%lf",theta(5,0),theta(6,0),theta(7,0));


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
        //ROS_INFO("m1 = %lf m2 = %lf",theta(0,0),theta(1,0));
        //ROS_INFO("fv1 = %lf fv2 = %lf fv3 = %lf",theta(2,0),theta(3,0),theta(4,0));
        //ROS_INFO("fc1 = %lf fc2 = %lf fc3 = %lf",theta(5,0),theta(6,0),theta(7,0));


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
            // 执行抓手合上动作
           //tol5 = 30;
            tol5 = 20;
            if(joint5_actual_angle>5.5)  //5.5 
            {
                tol5 = 0;
            }
            
        }
        else if (KB_D<-0.01)
        {
            // 执行抓手打开动作
           //tol5 = -45;
            tol5 = -35;
            //if(joint5_actual_angle<0.00001)
            if(joint5_actual_angle<0.3) //3  
            {
                tol5 = 0;
            }
        }
        else
        {
            tol5 = 0;
        }
        //ROS_INFO("joint5_actual_angle=%lf",joint5_actual_angle);
        send_message = joint5.MomentOutput(tol5);
        socketcan_send_pub.publish(send_message);



        frames.id = 0x80;
        frames.dlc = 0;
        socketcan_send_pub.publish(frames);
    }
}

//求向量符号函数
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

// Unity界面指令回调函数
void UnityCommand_Callback(const  std_msgs::Float64MultiArray& msg)
{
    //GripperFlag = msg.data[0];
    ForceFeedbackFlag = msg.data[0];
    MappingFlag = msg.data[1];  // 这个值先不用
    MovementFalg = msg.data[2];
}

/*
// 遥操作回调函数（适用于主端为Matlab下运行的Phantom系列主手）
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
*/
void MainpulatorCallback(const can_msgs::Frame &receive_message)
{
        switch (receive_message.id)
        {
        case 0x181:
            joint1.current_angle(receive_message);
            q(0, 0) = joint1.get_current_angle();
            //ROS_INFO("Joint1 = %lf", q(0, 0));
            break;
        case 0x182:
            joint2.current_angle(receive_message);
            q(1, 0) = joint2.get_current_angle();
            //ROS_INFO("Joint1 = %lf", q(2, 0));
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

void Omega7_PositionCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    //测试代码
    //ROS_INFO("Received position: x=%f, y=%f, z=%f", msg->vector.x, msg->vector.y, msg->vector.z);
    //实际使用代码
    Pm(0,0) = msg->vector.x;
    Pm(1,0) = msg->vector.y;
    Pm(2,0) = msg->vector.z;
    receive_omega_flag = true;
}
void Omega7_DirectionCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    //测试代码
    //ROS_INFO("Received direction: x=%f, y=%f, z=%f", msg->vector.x, msg->vector.y, msg->vector.z);
    //实际使用代码,末端姿态
    Drm(0,0) = msg->vector.x;
    Drm(1,0) = msg->vector.y;
    Drm(2,0) = msg->vector.z;
}
void Omega7_GripCallback(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO("Received grip state: %f", msg->data); // 夹持力,功能后续增加
    //根据Sigma7的抓手张开距离（完全合上为约0，完全打开约为0.027）来控制机械臂的张开和合上运动
    double Omega7_Gripper_distance = msg->data;
    if(Omega7_Gripper_distance < 0.01) // 完全合上
    {
        KB_D = 1;  //机械臂抓手合上指令
    }
    else{
        if(Omega7_Gripper_distance > 0.025) // 完全打开
        {
            KB_D = -1;  //机械臂抓手张开指令
        }
        else
        {
            KB_D = 0;  //保持静止
        }
    }
    
}
void Omega7_ButtonsCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
    //处理按钮数据回调,功能后续增加
    //ROS_INFO("Received buttons: button1=%d, button2=%d\n", msg->data[0], msg->data[1]);
}

// 主从末端位置映射
Vector3d PositionMapping(Vector3d& Pm)
{
    //Psd(0,0) = Kp(0,0)*Pm(0,0) + Tp(0,0);
    //Psd(1,0) = Kp(1,0)*Pm(1,0) + Tp(1,0);
    //Psd(2,0) = Kp(2,0)*Pm(2,0) + Tp(2,0);

    /*
    //x轴线性压缩映射 球形到半球
    double Rm = 0.1;  //Sigma7工作空间半径
    double Xm = (Pm(0,0) + Rm)/2;
    double scale = sqrt(1-(Xm/Rm)*(Xm/Rm));
    Psd(0,0) = (l1+l2)/Rm * Xm;
    Psd(1,0) = (l1+l2)/Rm * scale * Pm(1,0);
    Psd(2,0) = (l1+l2)/Rm * scale * Pm(2,0);
    */

    /*
    //体积保持映射 圆柱形到半球
    double R = 0.19/2;  //Sigma7工作空间半径
    double H = 0.13;  // Sigma7圆柱高度
    Psd(0,0) = 0.6*(l1+l2) * (std::pow(Pm(0,0)/H+0.5,1.0/3.0));
    double scale = sqrt(0.36*(l1+l2)*(l1+l2) - Psd(0,0)*Psd(0,0))/R;
    //double Rmm = sqrt(Xm*Xm + (Pm(1,0)*scale/Rm)*(Pm(1,0)*scale/Rm) +  (Pm(2,0)*scale/Rm)*(Pm(2,0)*scale/Rm));
    Psd(1,0) = scale * Pm(1,0);
    Psd(2,0) = scale * Pm(2,0);
    */
   /*
    double R = 0.19/2;  //Sigma7工作空间半径
    double H = 0.13;  // Sigma7圆柱高度
    double R1 = sqrt(R*R + (H/2)*(H/2));  //中间球体半径
    // 圆柱映射到球体
    Vector3d Psphere;
    double Pnorm = sqrt(Pm(0,0)*Pm(0,0)+Pm(1,0)*Pm(1,0)+Pm(2,0)*Pm(2,0));
    Psphere(0,0) = Pm(0,0) * R1 / Pnorm;
    Psphere(1,0) = Pm(1,0) * R1 / Pnorm;
    Psphere(2,0) = Pm(2,0) * R1 / Pnorm;
    //球体映射到x轴半球
    Psd(0,0) = ((l1+l2)/R1 * Psphere(0,0) + (l1+l2)) / 2;
    Psd(1,0) = (l1+l2)/R1 * Psphere(1,0) * sqrt(1-(Psd(0,0)/(l1+l2)) * (Psd(0,0)/(l1+l2)));
    Psd(2,0) = (l1+l2)/R1 * Psphere(2,0) * sqrt(1-(Psd(0,0)/(l1+l2)) * (Psd(0,0)/(l1+l2)));
    ROS_INFO("Mapping Position x=%lf, y = %lf, z=%lf", Psd(0,0), Psd(1,0), Psd(2,0));
    */
    // Sigma7工作空间近似一个半球，半径R=0.13m,但其坐标轴定义与机械臂不一致，需先进行坐标轴偏转
    double R = 0.13;
    double Tx = 0.1; //x轴偏转
    //以机械臂正前方往机械臂看的视角-映射
    //Psd(0,0) = 0.9*(l1+l2)/0.15 * (Pm(0,0)*cos(M_PI/4) - Pm(1,0)*sin(M_PI/4) + Tx);
    //Psd(1,0) = 0.9*(l1+l2)/R * (Pm(0,0)*sin(M_PI/4) + Pm(1,0)*cos(M_PI/4));
    //Psd(2,0) = 0.9*(l1+l2)/R * Pm(2,0);

    //以机械臂基座往前看的视角-映射
    Psd(0,0) = 0.9*(l1+l2)/0.15 * (-Pm(0,0)*cos(M_PI/4) + Pm(1,0)*sin(M_PI/4) + Tx);
    Psd(1,0) = 0.9*(l1+l2)/R * (-Pm(0,0)*sin(M_PI/4) - Pm(1,0)*cos(M_PI/4));
    Psd(2,0) = 0.9*(l1+l2)/R * Pm(2,0);
    ROS_INFO("Mapping Position x=%lf, y = %lf, z=%lf", Psd(0,0), Psd(1,0), Psd(2,0));
    return Psd;
}

//逆运动学求解
Vector3d ManipulatorInverseKinetic(Vector3d& Psd)
{
    expect_q_inv(0,0) = 0.0;
    expect_q_inv(1,0) = 0.0;
    expect_q_inv(2,0) = 0.0;
    int flag = 0;
    if(Psd(0,0) == 0.0) 
    {
        expect_q_inv(0,0) = 0.0;
        ROS_INFO("When x=0, q1 = 0!");
    }
    else
    {
        expect_q_inv(0,0) = atan(Psd(1,0)/Psd(0,0));
    }
    double rs;
    rs = sqrt(Psd(0,0)*Psd(0,0) + Psd(1,0)*Psd(1,0));
    double lx;
    lx = sqrt(Psd(0,0)*Psd(0,0) + Psd(1,0)*Psd(1,0) + Psd(2,0)*Psd(2,0));
    double lx0;
    lx0 = sqrt(l1*l1 + l2*l2 - 2*l1*l2*cos(M_PI/6));
    double qy;
    qy = atan(Psd(2,0)/rs);
    double qq;
    double qx;
    if(lx > (l1+l2))
    {
        qq = M_PI;
        qx = 0.0;
        flag = 1;
        
    }
    else{
        if(lx < lx0)
        {
            qq = M_PI/6;
            qx = acos((l1*l1 + lx0*lx0 - l2*l2)/(2*l1*lx0));
            flag = 2;
        }
        else{
            qq = acos((l1*l1 + l2*l2 - lx*lx)/(2*l1*l2));
            qx = acos((l1*l1 + lx*lx - l2*l2)/(2*l1*lx));
        }
    }
    /*
    if(rs>(l1+l2))
    {
        expect_q_inv(1,0) = 0.0;
        flag = 1;
        if(Psd(2,0) > (l1+l2))
        {
            expect_q_inv(2,0) = 0.0;
        }
    }
    else
    {
        expect_q_inv(2,0) = M_PI - qq;
        expect_q_inv(1,0) = qy + qx;
    }
    */
    expect_q_inv(2,0) = M_PI - qq;
    expect_q_inv(1,0) = qy + qx;
    if(flag == 1)
    {
        ROS_INFO("Beyond the workspace!");
    }
    if(flag==2)
    {
        ROS_INFO("Dangerous range!");
    }
    return expect_q_inv;
}

//关节轨迹规划, 返回机械臂的期望角度、角速度和角加速度
Matrix3d Joint_TrajectoryPlanning(Vector3d& expect_q_inv) 
{
    double t = getTimeSec();
    double ttt = ceil(t/0.0333333) * 0.0333333;
    //compensate_q00 = expect_q_inv(0,0);
    //compensate_q10 = expect_q_inv(1,0);
    //compensate_q20 = expect_q_inv(2,0);
    /*
    double dq_max=0.2;
    if ((q(0,0) - expect_q_inv(0,0))/(t-t_last) >= dq_max) // 主手变化太快
    {
        compensate_q00 = q(0,0) + dq_max*(t-t_last);
    }
    else if ((q(0,0) - expect_q_inv(0,0))/(t-t_last) <= -dq_max) 
    {
        compensate_q00 = q(0,0) - dq_max*(t-t_last);
    }
    else
    {
        compensate_q00 = expect_q_inv(0,0);
    }
    */
    //static  filter_expansion::planner::polynom_interpolator poly_planner(0.0335, 1);
	static  filter_expansion::planner::Bspline_interpolator Bspline_planner1(0.045, 3);
    Bspline_planner1.update(compensate_q00);
    q1_res_data = Bspline_planner1.output();
    q1_velocity = (q1_res_data - q1_result_last)/(t-t_last);
    //给关节角速度加个约束
    //if (q1_velocity >=0.2)
    //{
    //    q1_velocity = 0.2;
     //   q1_res_data = q1_result_last + q1_velocity*(t-t_last);
    //}
    //else if (q1_velocity <= -0.2)
    //{
    //     q1_velocity = -0.2;
    //     q1_res_data = q1_result_last + q1_velocity*(t-t_last);
    //}
    q1_acceleration = (q1_velocity - q1_velocity_last)/(t-t_last);
    //if (q1_acceleration >=3)
    //{
    //    q1_acceleration = 3;
    //}
    //else if (q1_acceleration <= -3)
    //{
    //    q1_acceleration = -3;
    //}
    /*
    if ((q(1,0) - expect_q_inv(1,0))/(t-t_last) >= dq_max) // 主手变化太快
    {
        compensate_q10 = q(1,0) + dq_max*(t-t_last);
    }
    else if ((q(1,0) - expect_q_inv(1,0))/(t-t_last) <= -dq_max) 
    {
        compensate_q10 = q(1,0) - dq_max*(t-t_last);
    }
    else
    {
        compensate_q10 = expect_q_inv(1,0);
    }
    */
    static  filter_expansion::planner::Bspline_interpolator Bspline_planner2(0.045, 3);
    Bspline_planner2.update(compensate_q10);
    q2_res_data = Bspline_planner2.output();
    q2_velocity = (q2_res_data - q2_result_last)/(t-t_last);
    q2_acceleration = (q2_velocity - q2_velocity_last)/(t-t_last);
 
    /*
    if ((q(2,0) - expect_q_inv(2,0))/(t-t_last) >= dq_max) // 主手变化太快
    {
        compensate_q20 = q(1,0) + dq_max*(t-t_last);
    }
    else if ((compensate_q20 - expect_q_inv(2,0))/(t-t_last) <= -dq_max) 
    {
        compensate_q20 = q(1,0) - dq_max*(t-t_last);
    }
    else
    {
        compensate_q20 = expect_q_inv(2,0);
    }   
    */
    static  filter_expansion::planner::Bspline_interpolator Bspline_planner3(0.045, 3);
    Bspline_planner3.update(compensate_q20);
    q3_res_data = Bspline_planner3.output();
    q3_velocity = (q3_res_data - q3_result_last)/(t-t_last);
    q3_acceleration = (q3_velocity - q3_velocity_last)/(t-t_last);


    // 数据迭代
    q1_result_last = q1_res_data;
    q1_velocity_last = q1_velocity;
    q2_result_last = q2_res_data;
    q2_velocity_last = q2_velocity;
    q3_result_last = q3_res_data;
    q3_velocity_last = q3_velocity;
    t_last = t;

    expect_q_traj(0,0) = q1_res_data;
    expect_q_traj(1,0) = q1_velocity;
    expect_q_traj(2,0) = q1_acceleration;
    expect_q_traj(0,1) = q2_res_data;
    expect_q_traj(1,1) = q2_velocity;
    expect_q_traj(2,1) = q2_acceleration;
    expect_q_traj(0,2) = q3_res_data;
    expect_q_traj(1,2) = q3_velocity;
    expect_q_traj(2,2) = q3_acceleration;

    return expect_q_traj;
 /****/
}

//关节约束规划器
/*输入：
原始角度：经过逆运动学求解后的值expect_q_inv
上周期规划后的机械臂的轨迹状态结构体last_state（包含角度、角速度、角加速度、加加速度）
输出：规划后的机械臂的轨迹状态结构体
*/
TrajectoryState ConstraitTrajectoryPlanning(Vector3d& expect_q_inv, TrajectoryState& last_state, double t)
{
   //计算约束边界
    Vector3d upper_bounds, lower_bounds;
    for(int i = 0; i < 3; i++)
    {
        double term1 = -3*LAMBDA_P*last_state.joint_a[i] - 3*pow(LAMBDA_P,2)*last_state.joint_v[i] 
              + pow(LAMBDA_P,3)*(Angle_LIMITS[i]-last_state.joint_p[i]);
        double term2 = -2*LAMBDA_V*last_state.joint_a[i] + pow(LAMBDA_V,2)*(AngleVelocity_LIMITS[i]-last_state.joint_v[i]);
        double term3 = LAMBDA_A*(AngleAcceleration_LIMITS[i]- last_state.joint_a[i]);       
        upper_bounds[i] = std::min({term1,term2,term3,Jerk_LIMITS[i]});

        term1 = -3*LAMBDA_P*last_state.joint_a[i] - 3*pow(LAMBDA_P,2)*last_state.joint_v[i] 
              + pow(LAMBDA_P,3)*(-Angle_LIMITS[i]-last_state.joint_p[i]);
        term2 = -2*LAMBDA_V*last_state.joint_a[i] + pow(LAMBDA_V,2)*(-AngleVelocity_LIMITS[i]-last_state.joint_v[i]);
        term3 = LAMBDA_A*(-AngleAcceleration_LIMITS[i] - last_state.joint_a[i]);
        lower_bounds[i] = std::max({term1,term2,term3,-Jerk_LIMITS[i]});
    }
    //ROS_INFO("upper_bounds=%lf,%lf,%lf",upper_bounds[0],upper_bounds[1],upper_bounds[2]);
    //ROS_INFO("lower_bounds=%lf,%lf,%lf",lower_bounds[0],lower_bounds[1],lower_bounds[2]);

    // 检查边界是否有效
    for(int i = 0; i < 3; i++)
    {
        if(lower_bounds[i] > upper_bounds[i]){
            ROS_ERROR("QP has no solutions: lower_bounds[%d] > upper_bounds[%d](%lf > %lf)", i,i,lower_bounds[i],upper_bounds[i]);
            return last_state;    // 返回上一状态值
        }
    }

    //设置QP问题（min 1/2*x'Hx+g'x)
    Vector3d gradient = 3*XI*last_state.joint_a + 3*pow(XI,2)*last_state.joint_v 
       + pow(XI,3)*(last_state.joint_p - expect_q_inv);
    real_t HH[9] = {1,0,0, 0,1,0, 0,0,1}; // 单位矩阵
    real_t AA[9] = {1,0,0, 0,1,0, 0,0,1}; // 单位矩阵
    real_t gg[3] = {gradient[0], gradient[1], gradient[2]};
    real_t lb[3] = {lower_bounds[0], lower_bounds[1], lower_bounds[2]};
    real_t ub[3] = {upper_bounds[0], upper_bounds[1], upper_bounds[2]};
    // 使用上周期的jerk作为初始猜测
    real_t x0[3] = {last_state.joint_jerk[0],last_state.joint_jerk[1],last_state.joint_jerk[2]};

   // 求解QP问题
    int_t nWSR = 50; //最大迭代次数
    QProblem qp(3,3);  //3变量，3约束
    returnValue ret = qp.init(
       HH,  //Hessian矩阵
       gg,  //梯度向量
       AA, //A矩阵（无约束矩阵）
       //nullptr,
       //nullptr,
       lb,  //下界
       ub,  //上界
       nullptr,  //lbA无约束下界
       nullptr,  //ubA无约束上界
       nWSR  //最大迭代次数
       //x0   //初始猜测
       );
   
   // 检查初始化是否成功
   if(ret != qpOASES::SUCCESSFUL_RETURN)
   {
       ROS_ERROR("QP Solver failed! The failed code is：%d",ret);
       return last_state;    // 返回上一状态值
   }
   else
   {
       //解决QP问题
       //ret = qp.hotstart(gg,lb,ub,nullptr,nullptr,nWSR);
       // 获取结果并验证
       real_t optimal_jerk[3];
       if(qp.getPrimalSolution(optimal_jerk) != qpOASES::SUCCESSFUL_RETURN)
       {
           ROS_ERROR("QP has no solution!!!!!");
           return last_state;    // 返回上一状态值
       }
       else
       {
           // 更新状态
           qp.getPrimalSolution(optimal_jerk);
           ROS_INFO("optimal_jerk = %lf,%lf,%lf",optimal_jerk[0], optimal_jerk[1], optimal_jerk[2]);
           Vector3d jerk_star(optimal_jerk[0], optimal_jerk[1], optimal_jerk[2]);

           TrajectoryState new_state;
           new_state.joint_jerk = jerk_star;
           new_state.joint_a = last_state.joint_a + jerk_star * t;
           new_state.joint_v = last_state.joint_v + new_state.joint_a * t;
           new_state.joint_p = last_state.joint_p + new_state.joint_v * t;
           return new_state;
       }      
       
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
    lambda1(0,0)  = 5;
    lambda1(1,1)  = 4;
    lambda1(2,2)  = 4.6; 
    lambda2(0,0)  = 27;
    lambda2(1,1)  = 11;
    lambda2(2,2)  = 31; 


    //lambda1(0,0)  = 7.5;
   // lambda1(1,1)  = 5;
    //lambda1(2,2)  = 4; 
    //lambda2(0,0)  = 120;
    //lambda2(1,1)  = 120;
    //lambda2(2,2)  = 92; 

    // for(int i=0;i<11;i++)
    //{
     //   theta_d(i,0) = 0;
   // }

    // 一开始ok的我的参数  
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

    // 娜姐实验的参数
    /*
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
    */
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
    gammamamm(0,0) = 500/10;
    gammamamm(1,1) = 100/10;

    gammamamm(2,2) = 1000/10;
    gammamamm(3,3) = 1200/10;
    gammamamm(4,4) = 1500/10;

    gammamamm(5,5) = 1500/10;
    gammamamm(6,6) = 1500/10;
    gammamamm(7,7) = 1500/10;

    gammamamm(8,8) = 1000/10;
    gammamamm(9,9) = 1000/10;
    gammamamm(10,10) = 1000/10;
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
    L(0,0)  = 15.6/5;
    L(1,1)  = 9.7/5;
    L(2,2) = 15.7/5; 

    // LDOB参数
    cutoff_freq = 15/5;

    // LESO参数
    //bandwidth = 30;//陆上的参数，稳定
    bandwidth = 15;//水下的参数，不稳定

    // NESO 参数
    beta1_ = 3*10;
    beta2_ = 3*10*10;
    beta3_ = 10*10*10;
    alpha1_ = 0.5;
    alpha2_ = 0.5;
    delta_ = 0.05;

    // FGM 参数
    K_gain = Matrix3d::Zero(3,3);
    K_gain.diagonal() << 10.0, 10.0, 10.0;

    // AFO 参数
    //w_AFO = 3;
    //a_AFO = 10;
    //lambda_AFO = 0;
    //eplsion_AFO = 7;
    //水下参数
    a_AFO = 20;
    lambda_AFO = 0;
    eplsion_AFO = 10;
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

    // 令M*dalpha1+C*alpha1+G+F+d = -fai^T * theta
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

// 线性扰动观测器，返回d_LDOB_hat
Vector3d LDOB(Vector3d& tol,Vector3d& q,Vector3d& dq,Vector3d& pre_dq,Vector3d& pre_d_LDOB_hat,double t)
{
    // Mn矩阵
    Matrix3d Mn_templete = Mn(q);
    // Cn矩阵
    Matrix3d Cn_templete = Cn(q, dq);
    // Gn矩阵
    Vector3d Gn_templete = Gn(q);
    // Fn 矩阵
    Vector3d Fn_templete = Fn(dq);

    ddq = (dq-pre_dq)/t;
    pre_dq = dq;

    double alpha = 2*M_PI*cutoff_freq*t;  //cutoff_freq截至频率 5-20Hz，更高截至频率更快响应但有更多噪声
    alpha = std::min(std::max(alpha, 0.0), 1.0); //限制在[0,1]之间

    //更新扰动估计
    pre_d_LDOB_hat = alpha * (Mn_templete * ddq + Cn_templete * dq + Gn_templete + Fn_templete - tol) + (1.0 - alpha) * pre_d_LDOB_hat;

    return pre_d_LDOB_hat;

}


// 非线性扰动观测器，返回d_NDOB_hat
Vector3d NDOB(Vector3d& tol,Vector3d& q,Vector3d& dq,double t)
{
    // Mn矩阵
    Matrix3d Mn_templete = Mn(q);
    // Cn矩阵
    Matrix3d Cn_templete = Cn(q, dq);
    // Gn矩阵
    Vector3d Gn_templete = Gn(q);
    // Fn 矩阵
    Vector3d Fn_templete = Fn(dq);
    
    double k = 1.0;
    // 固定增益L
    xi_dot = -L*xi-L*(-Cn_templete*dq-Gn_templete - Fn_templete + k*tol + L*Mn_templete*dq);  //rignt
    xi = xi + xi_dot * t;    
    d_NDOB_hat = xi + L * Mn_templete * dq;

    // 可变增益L_NDOB，数值没有变化
    //double L_NDOB_dot = -1*L_NDOB + 20*tanh(z1.transpose()*z1);
    //L_NDOB = L_NDOB + L_NDOB_dot * t;
    //xi_dot = -L_NDOB*xi-L_NDOB*(-Cn_templete*dq-Gn_templete - Fn_templete + k*tol + L_NDOB*Mn_templete*dq);  //rignt
    //xi = xi + xi_dot * t;    
    //d_NDOB_hat = xi + L_NDOB * Mn_templete * dq;

    //ROS_INFO("fx1=%f fx2=%f fx3=%f",Fn_templete(0,0),Fn_templete(1,0),Fn_templete(2,0));
    //ROS_INFO("G1=%f G2=%f G3=%f",Gn_templete(0,0),Gn_templete(1,0),Gn_templete(2,0));
    return  d_NDOB_hat;



    //xi_dot = -L*xi-L*(-Cn_templete*dq - fx +k*tol+L*Mn_templete*dq);   // without Gn
    //xi = xi + xi_dot * t;
    //d_NDOB_hat = xi + L * Mn_templete * dq  - Gn_templete/2 ;


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
    //d_NDOB_hat = xi + L * Mn_templete * dq  - Gn_templete/2 ;
    //u = d_NDOB_hat;
    
}

// 线性扩展状态观测器
Matrix3d LESO(Vector3d& tol,Vector3d& q,Vector3d& dq,Matrix3d& pre_x_LESO_hat,double t)
{
    // Mn矩阵
    Matrix3d Mn_templete = Mn(q);
    // Cn矩阵
    Matrix3d Cn_templete = Cn(q, dq);
    // Gn矩阵
    Vector3d Gn_templete = Gn(q);
    // Fn 矩阵
    Vector3d Fn_templete = Fn(dq);

    double beta1 = 3.0 * bandwidth;
    double beta2 = 3.0 * bandwidth * bandwidth;
    double beta3 = bandwidth * bandwidth * bandwidth;

    // 分离观测器状态
    Vector3d x1_hat = pre_x_LESO_hat.col(0);  // 位置估计
    Vector3d x2_hat = pre_x_LESO_hat.col(1);  // 速度估计
    Vector3d x3_hat = pre_x_LESO_hat.col(2);  // 外力估计

    // 计算观测误差
    Vector3d e1 = q - x1_hat;

    // ESO动态方程
    
    Vector3d x3_hat_dot = beta3 * e1;
    x3_hat = x3_hat + x3_hat_dot*t;

    Vector3d x2_hat_dot = Mn_templete.inverse()*(tol + x3_hat - Cn_templete * dq - Gn_templete - Fn_templete) + beta2 * e1;
    x2_hat = x2_hat + x2_hat_dot*t;

    Vector3d x1_hat_dot = x2_hat + beta1 * e1;
    x1_hat = x1_hat + x1_hat_dot*t;
    
    pre_x_LESO_hat << x1_hat, x2_hat, x3_hat;

    return pre_x_LESO_hat;

}


// 非线性扩展状态观测器  目前参数不收敛
Matrix3d NESO(Vector3d& tol,Vector3d& q,Vector3d& dq,Matrix3d& pre_x_NESO_hat,double t)
{
    // Mn矩阵
    Matrix3d Mn_templete = Mn(q);
    // Cn矩阵
    Matrix3d Cn_templete = Cn(q, dq);
    // Gn矩阵
    Vector3d Gn_templete = Gn(q);
    // Fn 矩阵
    Vector3d Fn_templete = Fn(dq);


    // 分离观测器状态
    Vector3d x1_hat_ = pre_x_NESO_hat.col(0);  // 位置估计
    Vector3d x2_hat_ = pre_x_NESO_hat.col(1);  // 速度估计
    Vector3d x3_hat_ = pre_x_NESO_hat.col(2);  // 外力估计

    // 计算观测误差
    Vector3d e1_ = q - x1_hat_;

    // ESO动态方程
    
    Vector3d x3_hat_dot_ = beta3_ * fal(e1_, 0.5, delta_);
    x3_hat_ = x3_hat_ + x3_hat_dot_ * t;

    Vector3d x2_hat_dot_ = x3_hat_  + beta2_ * fal(e1_, alpha2_, delta_) + Mn_templete.inverse()*(tol - Cn_templete * dq - Gn_templete - Fn_templete);
    x2_hat_ = x2_hat_ + x2_hat_dot_ * t;

    Vector3d x1_hat_dot_ = x2_hat_ + beta1_ * fal(e1_, alpha1_, delta_);
    x1_hat_ = x1_hat_ + x1_hat_dot_ * t;
    
    pre_x_NESO_hat << x1_hat_, x2_hat_, x3_hat_;

    return pre_x_NESO_hat;

}

// NESO中需要的fal函数
Vector3d fal(Vector3d& e_, double alpha_, double delta_)
{
    Vector3d g_;
    for (int i=0;i<3;i++)
    {
        if (fabs(e_(i,0)) > delta_)
        {
            g_(i,0) = pow(fabs(e_(i,0)), alpha_) * (e_(i,0) > 0 ? 1 : -1);
        }
        else
       {
            g_(i,0) = e_(i,0) / pow(delta_, 1.0 - alpha_);
       }
    }
    return g_;
}

// 传统（一阶）动量观测器
Vector3d FirstOrderGM(Vector3d& tol,Vector3d& q,Vector3d& dq, Vector3d& pre_integrand, double t)
{
    // Mn矩阵
    Matrix3d Mn_templete = Mn(q);
    // Cn矩阵
    Matrix3d Cn_templete = Cn(q, dq);
    // Gn矩阵
    Vector3d Gn_templete = Gn(q);
    // Fn 矩阵
    Vector3d Fn_templete = Fn(dq);
    
    //梯形积分
    Vector3d integrand = tol + d_FGM_hat + Cn_templete.transpose() * dq - Gn_templete - Fn_templete;
    p_hat = p_hat + (integrand + pre_integrand)/2 * t;
    pre_integrand = integrand;

    p = Mn_templete * dq;

    d_FGM_hat = K_gain * (p - p_hat);

    return d_FGM_hat;

}

// 自适应力观测器
Vector3d AFO(Vector3d& tol,Vector3d& q,Vector3d& dq, Vector3d& pre_d_AFO_hat, double t)
{
    // Mn矩阵
    Matrix3d Mn_templete = Mn(q);
    // Cn矩阵
    Matrix3d Cn_templete = Cn(q, dq);
    // Gn矩阵
    Vector3d Gn_templete = Gn(q);
    // Fn 矩阵
    Vector3d Fn_templete = Fn(dq);

    Vector3d x2_AFO = dq;
    Vector3d Omega_AFO = Mn_templete.inverse()*(tol - Cn_templete * dq - Gn_templete - Fn_templete);
    Matrix3d sigma_AFO = Mn_templete.inverse();
    
    // 给x2_AFO、Omega_AFO、sigma_AFO一阶低通滤波,
    // 滤波后的值为x2_AFO_star、Omega_AFO_star、sigma_AFO_star，定义为全局变量
    //double w_AFO = 2*M_PI*cutoff_freq*t;  //cutoff_freq截至频率 5-20Hz，更高截至频率更快响应但有更多噪声
    //w_AFO = std::min(std::max(w_AFO, 0.0), 1.0); //限制在[0,1]之间
    //double w_AFO = 0.5;
    double w_AFO = 0.2;  //水下的参数
    x2_AFO_star = w_AFO*x2_AFO + (1-w_AFO)*x2_AFO_star;
    Omega_AFO_star = w_AFO*Omega_AFO + (1-w_AFO)*Omega_AFO_star;
    sigma_AFO_star = w_AFO*sigma_AFO + (1-w_AFO)*sigma_AFO_star;
    /*if(is_first_sample)
    {
        x2_AFO_star = x2_AFO;
        Omega_AFO_star = Omega_AFO;
        sigma_AFO_star = sigma_AFO;
        is_first_sample = false;
    }
    else
    {
        x2_AFO_star = w_AFO*x2_AFO + (1-w_AFO)*x2_AFO_star;
        Omega_AFO_star = w_AFO*Omega_AFO + (1-w_AFO)*Omega_AFO_star;
        sigma_AFO_star = w_AFO*sigma_AFO + (1-w_AFO)*sigma_AFO_star;
    }*/
    // 求解中间自适应律
    Matrix3d Gamma1_AFO_dot = -a_AFO * Gamma1_AFO + sigma_AFO_star.transpose()*sigma_AFO_star;
    Gamma1_AFO = Gamma1_AFO_dot * t + Gamma1_AFO;
    Vector3d Gamma2_AFO_dot = -a_AFO * Gamma2_AFO + sigma_AFO_star.transpose()*((x2_AFO-x2_AFO_star)/w_AFO - Omega_AFO_star);
    Gamma2_AFO = Gamma2_AFO_dot * t + Gamma2_AFO;

    Vector3d Pai1_AFO = Gamma1_AFO * pre_d_AFO_hat - Gamma2_AFO;
    Vector3d Pai2_AFO = sigma_AFO_star.transpose()*sigma_AFO_star*pre_d_AFO_hat - sigma_AFO_star.transpose()*((x2_AFO-x2_AFO_star)/w_AFO - Omega_AFO_star);
    
    // 增益学习律
    Matrix3d kappa_AFO_dot = a_AFO*kappa_AFO - kappa_AFO*sigma_AFO_star.transpose()*sigma_AFO_star*kappa_AFO;
    kappa_AFO = kappa_AFO + kappa_AFO_dot*t;

    // 外力估计律
    Vector3d d_AFO_hat_dot = -eplsion_AFO * kappa_AFO * (Pai1_AFO + lambda_AFO * Pai2_AFO);
    d_AFO_hat = pre_d_AFO_hat + d_AFO_hat_dot * t;

    return d_AFO_hat;

}

// 计算末端力
Vector3d ComputeEndForce(Vector3d& tol_e,Matrix3d& Jaco)
{
    Matrix3d JT = Jaco.transpose();   
    //奇异值分解
    JacobiSVD<Matrix3d> svd(JT, ComputeFullU | ComputeFullV);

    //条件数计算
    double cond = svd.singularValues()(0) / svd.singularValues()(2);


    ROS_INFO("cond = %lf", cond);
    ROS_INFO("vd.singularValues()(2) = %lf", svd.singularValues()(2));
    // 判断是否接近奇异位置
    bool near_singular = (cond > 1e3) || (svd.singularValues()(2) < 1e-3);

    if(!near_singular)
    {
        //非奇异，使用伪逆
        Fe = svd.solve(tol_e);
        ROS_INFO("no_near_singular!!");
    }
    else
    {
        //奇异：阻尼最小二乘法
        ROS_INFO("near_singular!!");
        Matrix3d I = Matrix3d::Identity();
        Fe = Jaco * (JT * Jaco + 0.01 * 0.01 * I).inverse() * tol_e;
        //Fe = (JT * Jaco).inverse() * JT * tol_e;
    }
    Fe = Fe.cwiseMin(100.0).cwiseMax(-100.0);
    return Fe;
}


// Mn矩阵计算
Matrix3d Mn(Vector3d& q)
{
    // Mn矩阵
    Matrix3d Mn_templete;
    //m1 = theta(0,0);
    //m2 = theta(1,0);
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

    return Mn_templete;
}
// Cn矩阵计算
Matrix3d Cn(Vector3d& q,Vector3d& dq)
{
    // Cn矩阵
    Matrix3d Cn_templete;
    //m1 = theta(0,0);
    //m2 = theta(1,0);
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

    return Cn_templete;
}
// Gn矩阵计算
Vector3d Gn(Vector3d& q)
{
    // Gn矩阵
    Vector3d Gn_templete;
    //m1 = theta(0,0);
    //m2 = theta(1,0);
    Gn_templete(0,0)=0;
    Gn_templete(1,0)=(1.0/2.0 * m1 + m2) * g * l1 * cos(q(1,0)) + 1.0/2.0 * m2 * g * l2 * cos(q(1,0)-q(2,0));
    Gn_templete(2,0)= -1.0/2.0 * m2 * g * l2 * cos(q(1,0)-q(2,0));

    return Gn_templete;
}
// Fn矩阵计算
Vector3d Fn(Vector3d& dq)
{
    // Fn 矩阵
    Vector3d Fn_templete;
    //fv(0,0) = theta(2,0);
    //fv(1,0) = theta(3,0);
    //fv(2,0) = theta(4,0);
    //fc(0,0) = theta(5,0);
    //fc(1,0) = theta(6,0);
    //fc(2,0) = theta(7,0);
    Fn_templete(0,0) = fv(0,0) * dq(0,0) + fc(0,0) * atan2(900*dq(0,0),1)*2.0/(M_PI*1.0);
    Fn_templete(1,0) = fv(1,0) * dq(1,0) + fc(1,0) * atan2(900*dq(1,0),1)*2.0/(M_PI*1.0);
    Fn_templete(2,0) = fv(2,0) * dq(2,0) + fc(2,0) * atan2(900*dq(2,0),1)*2.0/(M_PI*1.0);

    return Fn_templete;
}

// 机械臂雅可比矩阵
Matrix3d Jn(Vector3d& q)
{
    Matrix3d Jn_templete;
    Jn_templete(0,0) = -(l1 * cos(q(1,0)) + l2 * cos(q(1,0)-q(2,0)) ) * sin(q(0,0));
    Jn_templete(0,1) = -(l1 * sin(q(1,0)) + l2 * sin(q(1,0)-q(2,0)) ) * cos(q(0,0));
    Jn_templete(0,2) = l2 * sin(q(1,0)-q(2,0) * cos(q(0,0)));
    Jn_templete(1,0) = (l1 * cos(q(1,0)) + l2 * cos(q(1,0)-q(2,0)) ) * cos(q(0,0));
    Jn_templete(1,1) = -(l1 * sin(q(1,0)) + l2 * sin(q(1,0)-q(2,0)) ) * sin(q(0,0));
    Jn_templete(1,2) = l2 * sin(q(1,0)-q(2,0) * sin(q(0,0)));
    Jn_templete(2,0) = 0;
    Jn_templete(2,1) = l1 * cos(q(1,0)) + l2 * cos(q(1,0)-q(2,0));
    Jn_templete(2,2) = -l2 * cos(q(1,0)-q(2,0));

    return Jn_templete;
}
