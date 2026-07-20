// ReadMe:
//此程序可以实现接收Matlab/Simulink发送的机械臂期望角速度、角速度和角加速度，进行自适应控制，
//并发送给Matlab/Simulink实际角度信息
//接收来自ZWT双目相机识别的小车信息并发送小车信息以及机械臂信息给Unity
//针对主端设备为Phantom Omni/Preium力反馈手，与Matlab进行ROS通信
//保证主从设备上位机以及Unity程序运行电脑在同一局域网，其能ping通
//与Unity通信依赖ROS-TCP-Endpoint包，请确保此程序包正确下载到src文件夹下，并在程序运行时开启endpoint.launch结点
//新增模仿学习通信接口，通过话题与上位机（如Windows+ros_tcp_endpoint）通信
//运行前先将文件名改为test_node.cpp再编译运行

#include "mainpulator/mainpulator_param.h"
#include "mainpulator/mainpulator_control.h"
#include "mainpulator/motion_csv_logger.h"
#include "mainpulator/online_joint_planner.h"
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <memory>
#include <vector>
//用到传9消息,需要一个合适的消息类型
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>


#define PI acos(-1)
using namespace std;
using namespace Eigen;
using mainpulator_motion::JointVector;

enum class ControlMode
{
    Torque,
    Position
};

string control_type;
ControlMode control_mode = ControlMode::Torque;
string active_control_source = "idle";
double torque_home_duration = 5.0;
double runtime_motion_duration = 5.0;
double runtime_motion_report_duration = 5.0;
bool trajectory_planner_enabled = false;
std::unique_ptr<mainpulator_motion::OnlineJointPlanner> online_joint_planner;
bool online_joint_planner_initialized = false;
JointVector raw_target_q {{0.0, 0.0, 0.0, 0.0}};
JointVector raw_target_dq {{0.0, 0.0, 0.0, 0.0}};
JointVector raw_target_ddq {{0.0, 0.0, 0.0, 0.0}};
double expect_q4_velocity = 0.0;
double expect_q4_acceleration = 0.0;
std::string planner_result = "disabled";
uint32_t latest_command_ind = 0;
std::string latest_command_source = "idle";
bool motion_data_log_enabled = false;
std::string motion_data_log_path = "/home/night/robot/logs/arm_motion.csv";
std::unique_ptr<mainpulator_motion::MotionCsvLogger> motion_csv_logger;
bool startup_homing_active = false;
bool joint_position_received[3] = {false, false, false};
bool joint_velocity_received[3] = {false, false, false};
Vector3d torque_home_start_q;

struct RuntimeMotionState
{
    bool active = false;
    uint32_t ind = 0;
    std::string source;
    ros::WallTime start_time;
    Vector3d coefficient[6];
    double last_status_progress = -1.0;
};

RuntimeMotionState runtime_motion;
bool runtime_motion_id_valid = false;
uint32_t last_runtime_motion_ind = 0;
std::string last_runtime_motion_source;
ros::Time last_runtime_motion_stamp;
ros::Publisher motion_status_pub;
ros::NodeHandle* private_node_handle = NULL;
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
double l2 = 0.424;
//Matrix3Xd Mn(3,3);
//Matrix3Xd Cn(3,3);
//Vector3d Gn;

//g根据安装方向修改 橙色底座为-------
double g = 9.8;
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
Vector3d Ps;  //机械臂末端位置
Vector3d car_position2base; //双目相机识别的小车相对于机械臂基座的位置

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

Vector3d zerovector;

// 定义接收到zwt双目相机识别的小车位置数据
Vector3d car_position;  //双目相机识别的小车相对于双目相机的位置
double gripper_flag=0.0; 

// 定义一个虚拟力反馈
Vector3d ForceFeedback;

//--------------------------------------------------------------------------------------------------------------------

void  AdaptiveBACKSTEPPINGparam_init();

// 控制器
Vector3d AdaptiveBackstepping(Vector3d& expect_q, Vector3d& expect_dq, Vector3d& expect_ddq, Vector3d& q, Vector3d& dq, Vector3d& tol_e_hat, double t);


void MainpulatorCallback(const can_msgs::Frame &receive_message);
/** * @brief 位置模式回调函数,编码器返回速度、位置等信息
 * @param receive_message ros_canopen类对象
 * @return 
 */

void TeleOperationCallback(const  sensor_msgs::Imu& msg);
void H5TeleOperationCallback(const sensor_msgs::Imu& msg);
void ActiveControlSourceCallback(const std_msgs::String& msg);
bool ApplyLogicalArmCommand(const sensor_msgs::Imu& msg, const std::string& source);
void StartRuntimeMotion(const sensor_msgs::Imu& msg, const std::string& source);
void CancelRuntimeMotion(const std::string& state);
void UpdateRuntimeMotion();
bool InitializeOnlinePlannerFromReference();
void RecordMotionLogSample(std::uint64_t loop_index, double dt_sec);
void PublishMotionStatus(const std::string& state, double progress, uint32_t ind, const std::string& source);

void CarPosition_Callback(const  std_msgs::Float64MultiArray& msg);

// 【新增代码】模仿学习回调函数的前向声明
void ImitationCallback(const sensor_msgs::JointState::ConstPtr& msg);

Vector3d VirtualForceGeneration(Vector3d& q, Vector3d& car_position);

bool TorqueFeedbackReady()
{
    for (int i = 0; i < 3; ++i)
    {
        if (!joint_position_received[i] || !joint_velocity_received[i])
        {
            return false;
        }
    }
    return true;
}

int main(int argc, char *argv[])
{
    // demoKinematics();
    //实际角度初始化
    q.setZero();
    dq.setZero();
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
    nh.param<std::string>("control_type", control_type, "torque");
    std::transform(control_type.begin(), control_type.end(), control_type.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (control_type == "pid")
    {
        ROS_WARN("control_type=PID is deprecated; using position mode");
        control_type = "position";
    }
    if (control_type == "torque")
    {
        control_mode = ControlMode::Torque;
    }
    else if (control_type == "position")
    {
        control_mode = ControlMode::Position;
    }
    else
    {
        ROS_FATAL("Invalid control_type '%s'; expected torque or position", control_type.c_str());
        return 2;
    }
    nh.param("torque_home_duration", torque_home_duration, 5.0);
    nh.param("runtime_motion_duration", runtime_motion_duration, 5.0);
    nh.param("trajectory_planner_enabled", trajectory_planner_enabled, false);
    std::vector<double> trajectory_max_velocity;
    std::vector<double> trajectory_max_acceleration;
    std::vector<double> trajectory_max_jerk;
    nh.param("trajectory_max_velocity", trajectory_max_velocity, std::vector<double>(4, 0.5));
    nh.param("trajectory_max_acceleration", trajectory_max_acceleration, std::vector<double>(4, 1.0));
    nh.param("trajectory_max_jerk", trajectory_max_jerk, std::vector<double>(4, 5.0));
    nh.param("motion_data_log_enabled", motion_data_log_enabled, false);
    nh.param<std::string>("motion_data_log_path", motion_data_log_path,
                          "/home/night/robot/logs/arm_motion.csv");
    int motion_data_log_queue_capacity = 8192;
    int motion_data_log_flush_rows = 100;
    nh.param("motion_data_log_queue_capacity", motion_data_log_queue_capacity, 8192);
    nh.param("motion_data_log_flush_rows", motion_data_log_flush_rows, 100);
    if (control_mode == ControlMode::Torque && torque_home_duration <= 0.0)
    {
        ROS_FATAL("torque_home_duration must be greater than zero");
        return 2;
    }
    if (runtime_motion_duration <= 0.0)
    {
        ROS_FATAL("runtime_motion_duration must be greater than zero");
        return 2;
    }
    if (trajectory_planner_enabled && control_mode != ControlMode::Torque)
    {
        ROS_FATAL("trajectory_planner_enabled requires control_type=torque");
        return 2;
    }
    if (trajectory_planner_enabled)
    {
        if (trajectory_max_velocity.size() != 4 || trajectory_max_acceleration.size() != 4 ||
            trajectory_max_jerk.size() != 4)
        {
            ROS_FATAL("trajectory limits must each contain exactly four values");
            return 2;
        }
        JointVector max_velocity;
        JointVector max_acceleration;
        JointVector max_jerk;
        std::copy(trajectory_max_velocity.begin(), trajectory_max_velocity.end(), max_velocity.begin());
        std::copy(trajectory_max_acceleration.begin(), trajectory_max_acceleration.end(), max_acceleration.begin());
        std::copy(trajectory_max_jerk.begin(), trajectory_max_jerk.end(), max_jerk.begin());
        online_joint_planner.reset(new mainpulator_motion::OnlineJointPlanner(0.01));
        std::string planner_error;
        if (!online_joint_planner->configure(max_velocity, max_acceleration, max_jerk, &planner_error))
        {
            ROS_FATAL("Invalid trajectory planner configuration: %s", planner_error.c_str());
            return 2;
        }
        planner_result = "startup_homing";
    }
    if (motion_data_log_enabled)
    {
        if (motion_data_log_queue_capacity <= 0 || motion_data_log_flush_rows <= 0)
        {
            ROS_ERROR("Motion data logging disabled: queue capacity and flush rows must be positive");
            motion_data_log_enabled = false;
        }
        else
        {
            mainpulator_motion::MotionCsvLoggerConfig logger_config;
            logger_config.path = motion_data_log_path;
            logger_config.queue_capacity = static_cast<std::size_t>(motion_data_log_queue_capacity);
            logger_config.flush_rows = static_cast<std::size_t>(motion_data_log_flush_rows);
            motion_csv_logger.reset(new mainpulator_motion::MotionCsvLogger());
            std::string logger_error;
            if (!motion_csv_logger->start(logger_config, &logger_error))
            {
                ROS_ERROR("Motion data logging disabled: %s", logger_error.c_str());
                motion_csv_logger.reset();
                motion_data_log_enabled = false;
            }
        }
    }
    private_node_handle = &nh;
    startup_homing_active = (control_mode == ControlMode::Torque);
    nh.setParam("startup_homing_complete", false);
    nh.setParam("runtime_motion_complete", true);
    //控制器发布数据至机械臂
    // ROS_INFO("control_type = %s", control_type.c_str());
    ros::Publisher socketcan_send_pub = socketcan_send.advertise<can_msgs::Frame>("sent_messages", 10);

    //控制接收机械臂数据
    socketcan_receive_sub = socketcan_receive.subscribe("received_messages", 1000, MainpulatorCallback);

    // 此订阅器和发布器针对主端Phantom Omni/Preium力反馈手，与Matlab通信
    // 订阅器TeleOperation_receive_sub订阅遥操作期望信息，话题为"/pub_joint_state"，回调函数为TeleOperationCallback
    ros::NodeHandle TeleOperation;
    ros::Subscriber TeleOperation_receive_sub = TeleOperation.subscribe("/pub_joint_state",10, TeleOperationCallback);  
    ros::Subscriber H5_TeleOperation_receive_sub = TeleOperation.subscribe("/h5/pub_joint_state", 10, H5TeleOperationCallback);
    ros::Subscriber active_control_source_sub = TeleOperation.subscribe("/arm/active_control_source", 10, ActiveControlSourceCallback);
    // 发布信息给Matlab
    ros::Publisher TeleOperation_pub = TeleOperation.advertise<sensor_msgs::JointState>("/Matlab/dataplot",10);

    
    // 发送给zwt的Unity头盔机械臂的三个关节角度和双目相机识别的小车位置
    ros::NodeHandle zwt;
    ros::Publisher pubUnity2ZWT = zwt.advertise<std_msgs::Float64MultiArray>("/JXB_data", 10);
    ros::Publisher pubUnity2ZWT2 = zwt.advertise<std_msgs::Float64MultiArray>("car_xyz", 10);
    // 接收zwt双目相机的数据
    ros::Subscriber Steroes_sub_ = zwt.subscribe("/car/position", 10, CarPosition_Callback);

    // 【新增代码】为模仿学习创建发布器和订阅器
    // 1. 发布机械臂当前状态给 AI PC
    ros::Publisher imitation_state_pub = nh.advertise<sensor_msgs::JointState>("/robot/imitation_state", 10);
    motion_status_pub = nh.advertise<std_msgs::String>("/arm/motion_status", 10, true);
    // 2. 订阅来自 AI PC 的期望轨迹
    ros::Subscriber imitation_trajectory_sub = nh.subscribe<sensor_msgs::JointState>("/imitation/desired_trajectory", 10, ImitationCallback);


    expect_q.setZero();
    expect_dq.setZero();
    expect_ddq.setZero();
    zerovector.setZero();

    const bool torque_control = (control_mode == ControlMode::Torque);
    ros::Rate loop_rate(torque_control ? 100.0 : 20.0);
    int socket_can = param::SocketCANInit();
    // ROS_INFO("socket_can = %d", socket_can);
    // ROS_INFO("control_type  =  %s", control_type.c_str());

    can_msgs::Frame send_message;

    //机械臂输出化配置
    // ROS_INFO("init");
    if (torque_control)
    {
        // Strictly preserve the original 1228 torque-loop initialization order.
        param::MomentInit(joint5, socket_can);
        param::MomentInit(joint1, socket_can);
        param::MomentInit(joint2, socket_can);
        param::MomentInit(joint3, socket_can);
        param::PositionInit(joint4, socket_can);
    }
    else
    {
        param::PositionInit(joint1, socket_can);
        param::PositionInit(joint2, socket_can);
        param::PositionInit(joint3, socket_can);
        param::PositionInit(joint4, socket_can);
        param::MomentInit(joint5, socket_can);
    }
    usleep(500000);

    if (torque_control)
    {
        // Strictly preserve the original 1228 torque-loop enable order.
        param::Enable(socket_can, 6, joint1);
        param::Enable(socket_can, 6, joint2);
        param::Enable(socket_can, 6, joint3);
        param::Enable(socket_can, 6, joint5);

        // J4 is the merged position-loop extension and stays after the legacy
        // J1-J3/J5 torque-loop sequence.
        param::Enable(socket_can, 6, joint4);
        usleep(500000);

        // Feedback capture and homing planning are layered after the complete
        // legacy initialization/enable sequence.
        const ros::WallTime feedback_deadline = ros::WallTime::now() + ros::WallDuration(3.0);
        while (ros::ok() && !TorqueFeedbackReady() && ros::WallTime::now() < feedback_deadline)
        {
            can_msgs::Frame sync_frame;
            sync_frame.id = 0x80;
            sync_frame.dlc = 0;
            socketcan_send_pub.publish(sync_frame);
            ros::spinOnce();
            ros::WallDuration(0.01).sleep();
        }
        if (!TorqueFeedbackReady())
        {
            ROS_FATAL("Torque startup feedback timeout after legacy enable sequence: position=[%d,%d,%d] velocity=[%d,%d,%d]",
                      joint_position_received[0], joint_position_received[1], joint_position_received[2],
                      joint_velocity_received[0], joint_velocity_received[1], joint_velocity_received[2]);
            return 3;
        }
        torque_home_start_q = q;
        expect_q = torque_home_start_q;
        expect_dq.setZero();
        expect_ddq.setZero();
        // ROS_INFO("Torque startup position captured: [%f, %f, %f]",
        //          torque_home_start_q(0,0), torque_home_start_q(1,0), torque_home_start_q(2,0));
    }

    if (!torque_control)
    {
        param::Enable(socket_can, 6, joint1);
        param::Enable(socket_can, 6, joint2);
        param::Enable(socket_can, 6, joint3);
        param::Enable(socket_can, 6, joint4);
        param::Enable(socket_can, 6, joint5);
        usleep(500000);
    }
    // ROS_INFO("init end");

    ros::WallTime torque_home_start_time;
    if (torque_control)
    {
        torque_home_start_time = ros::WallTime::now();
        // ROS_INFO("Starting %.3f second quintic torque-mode homing trajectory", torque_home_duration);
    }
    else
    {
        nh.setParam("startup_homing_complete", true);
        // ROS_INFO("Position mode initialization complete");
    }

    AdaptiveBACKSTEPPINGparam_init();

    // 摩擦参数初始化
    fc(0,0)  =  7.65;
    fc(1,0)  = 7.738;
    fc(2,0) =  6.319; 
    fv(0,0)  =  19.6233;
    fv(1,0)  = 13.758;
    fv(2,0) = 12.865; 

    std::uint64_t loop_index = 0;
    ros::WallTime previous_loop_time = ros::WallTime::now();
    while (ros::ok())
    {
        const ros::WallTime loop_time = ros::WallTime::now();
        const double loop_dt_sec = (loop_time - previous_loop_time).toSec();
        previous_loop_time = loop_time;
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

        if (torque_control && startup_homing_active)
        {
            const double elapsed = (ros::WallTime::now() - torque_home_start_time).toSec();
            const double tau = std::max(0.0, std::min(1.0, elapsed / torque_home_duration));
            const double tau2 = tau * tau;
            const double tau3 = tau2 * tau;
            const double tau4 = tau3 * tau;
            const double tau5 = tau4 * tau;
            const double scale = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
            const double scale_dot = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / torque_home_duration;
            const double scale_ddot = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) /
                                      (torque_home_duration * torque_home_duration);

            expect_q = torque_home_start_q * (1.0 - scale);
            expect_dq = torque_home_start_q * (-scale_dot);
            expect_ddq = torque_home_start_q * (-scale_ddot);

            if (elapsed >= torque_home_duration)
            {
                expect_q.setZero();
                expect_dq.setZero();
                expect_ddq.setZero();
                startup_homing_active = false;
                nh.setParam("startup_homing_complete", true);
                // ROS_INFO("Torque-mode startup homing trajectory complete");
            }
        }

        if (!startup_homing_active)
        {
            UpdateRuntimeMotion();
        }

        RecordMotionLogSample(loop_index++, loop_dt_sec);
        
        // 【新增代码】发布机械臂的当前关节状态给模仿学习节点
        sensor_msgs::JointState current_imitation_state;
        current_imitation_state.header.stamp = ros::Time::now();
        current_imitation_state.position.resize(5);
        current_imitation_state.velocity.resize(5);
        
        current_imitation_state.position[0] = -q(0,0); // logical J1 = -physical J1
        current_imitation_state.position[1] = q(1,0);
        current_imitation_state.position[2] = q(2,0);
        current_imitation_state.position[3] = joint4_actual_angle;
        current_imitation_state.position[4] = joint5_actual_angle;

        current_imitation_state.velocity[0] = -dq(0,0);
        current_imitation_state.velocity[1] = dq(1,0);
        current_imitation_state.velocity[2] = dq(2,0);
        current_imitation_state.velocity[3] = joint4_actual_velocity;
        current_imitation_state.velocity[4] = 0.0;
        
        imitation_state_pub.publish(current_imitation_state);
        

        // 发布给主端Matlab角度、角速度、虚拟力反馈
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.position.resize(9);
        joint_state.velocity.resize(9);
        joint_state.effort.resize(15);

        joint_state.position[0] = KB_D; 
        joint_state.position[1] = joint5_actual_angle;
        joint_state.position[2] = joint4_actual_angle;
        joint_state.position[3] = expect_q(0,0); // 轨迹规划后的期望关节角度
        joint_state.position[4] = expect_q(1,0);
        joint_state.position[5] = expect_q(2,0);
        joint_state.position[6] = q(0,0);  //机械臂实际关节角度
        joint_state.position[7] = q(1,0);
        joint_state.position[8] = q(2,0);
        joint_state.position[3] = -joint_state.position[3];
        joint_state.position[6] = -joint_state.position[6];

        joint_state.velocity[0] = expect_dq(0,0); // 轨迹规划后的期望关节角速度
        joint_state.velocity[1] = expect_dq(1,0);
        joint_state.velocity[2] = expect_dq(2,0);
        joint_state.velocity[3] = dq(0,0);  //机械臂实际关节角速度
        joint_state.velocity[4] = dq(1,0);  
        joint_state.velocity[5] = dq(2,0);
        joint_state.velocity[6] = expect_ddq(0,0);// 轨迹规划后的期望关节角加速度
        joint_state.velocity[7] = expect_ddq(1,0);  
        joint_state.velocity[8] = expect_ddq(2,0);
        joint_state.velocity[0] = -joint_state.velocity[0];
        joint_state.velocity[3] = -joint_state.velocity[3];
        joint_state.velocity[6] = -joint_state.velocity[6];

        ForceFeedback = VirtualForceGeneration(q, car_position);
        joint_state.effort[0] = ForceFeedback(0,0); // 虚拟力反馈
        joint_state.effort[1] = ForceFeedback(1,0);
        joint_state.effort[2] = ForceFeedback(2,0);

        joint_state.effort[3] = theta(8,0);
        joint_state.effort[4] = theta(9,0);
        joint_state.effort[5] = theta(10,0);

        joint_state.effort[6] = expect_q(0,0)-q(0,0);
        joint_state.effort[7] = expect_q(1,0)-q(1,0);
        joint_state.effort[8] = expect_q(2,0)-q(2,0);        
        joint_state.effort[6] = -joint_state.effort[6];


        TeleOperation_pub.publish(joint_state);

        /// 发送给zwt的Unity头盔机械臂的三个关节角度和双目相机识别的小车位置
        std_msgs::Float64MultiArray array_msg1;
        array_msg1.data.push_back(-q(0,0)); // logical J1
        array_msg1.data.push_back(q(1,0));
        array_msg1.data.push_back(q(2,0));
        array_msg1.data.push_back(gripper_flag);  //抓手标志位
        pubUnity2ZWT.publish(array_msg1);
        std_msgs::Float64MultiArray array_msg2;
        array_msg2.data.push_back(car_position(0,0));
        array_msg2.data.push_back(car_position(1,0));
        array_msg2.data.push_back(car_position(2,0));
        pubUnity2ZWT2.publish(array_msg2);

    

        // 根据启动参数选择 J1-J3 力矩环或位置环。
        // ROS_INFO("q_e=[%f,%f,%f,%f]", expect_q(0,0), expect_q(1,0), expect_q(2,0), KB_D);
        //ROS_INFO("dq_e=[%f,%f,%f]", expect_dq(0,0), expect_dq(1,0), expect_dq(2,0));
        //ROS_INFO("car_position = %lf, %lf, %lf", car_position(0,0), car_position(1,0), car_position(2,0));
        // ROS_INFO("q = %lf, %lf, %lf", q(0,0), q(1,0), q(2,0));
        can_msgs::Frame frames;
        if (torque_control)
        {
            tol = AdaptiveBackstepping(expect_q, expect_dq, expect_ddq, q, dq, zerovector, 0.002);

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
            send_message = joint1.MomentOutput(tol(0,0));
            socketcan_send_pub.publish(send_message);
            send_message = joint2.MomentOutput(tol(1,0));
            socketcan_send_pub.publish(send_message);
            send_message = joint3.MomentOutput80(tol(2,0));
            socketcan_send_pub.publish(send_message);
        }
        else
        {
            send_message = joint1.set_angle_for_new_joint(expect_q(0,0));
            socketcan_send_pub.publish(send_message);
            send_message = joint1.set_angle(expect_q(0,0));
            socketcan_send_pub.publish(send_message);

            send_message = joint2.set_angle_for_new_joint(expect_q(1,0));
            socketcan_send_pub.publish(send_message);
            send_message = joint2.set_angle(expect_q(1,0));
            socketcan_send_pub.publish(send_message);

            send_message = joint3.set_angle_for_new_joint(expect_q(2,0));
            socketcan_send_pub.publish(send_message);
            send_message = joint3.set_angle(expect_q(2,0));
            socketcan_send_pub.publish(send_message);
        }

        // 关节 4位置控制 joint4angle其实是期望角度
        if(joint4angle<-M_PI)
        {
            joint4angle=-M_PI;
        }
        if(joint4angle>M_PI)
        {
            joint4angle=M_PI;     
        }
        send_message = joint4.set_angle_for_new_joint(joint4angle);
        socketcan_send_pub.publish(send_message);
        send_message =joint4.set_angle(joint4angle);
        socketcan_send_pub.publish(send_message);
        //ROS_INFO("joint4angle=%lf",joint4angle);

        // 关节 5力矩控制
        if(KB_D>0.01)
        {
            // 执行抓手合上动作
           //tol5 = 30;
            tol5 = 30;
            if(joint5_actual_angle>4.5)  //插拔件6.2 10mm时对应的角度值
            {
                tol5 = 0;
                gripper_flag = 1; //说明已经抓到东西
            }
            
        }
        else if (KB_D<-0.01)
        {
            // 执行抓手打开动作
           //tol5 = -45;
            tol5 = -30;
            //if(joint5_actual_angle<0.00001)
            if(joint5_actual_angle<0.8) //张开 72mm
            {
                tol5 = 0;
            }
            gripper_flag = 0; //执行抓手打开动作，认为没有抓到东西
        }
        else
        {
            tol5 = 0;
        }
        // ROS_INFO("joint5_actual_angle=%lf",joint5_actual_angle);
        //ROS_INFO("gripper_flag=%lf",gripper_flag);
        send_message = joint5.MomentOutput(tol5);
        socketcan_send_pub.publish(send_message);



        frames.id = 0x80;
        frames.dlc = 0;
        socketcan_send_pub.publish(frames);
        loop_rate.sleep();
    }
    if (motion_csv_logger)
    {
        motion_csv_logger->stop();
    }
    return 0;
}


// 接收到双目识别小车位置的回调函数
void CarPosition_Callback(const  std_msgs::Float64MultiArray& msg)
{
    car_position(0,0)=msg.data[0]/1000;  
    car_position(1,0)=msg.data[1]/1000;
    car_position(2,0)=msg.data[2]/1000;
    // ROS_INFO("car_position = %lf, %lf, %lf", car_position(0,0), car_position(1,0), car_position(2,0));
}

// 虚拟力反馈生成
Vector3d VirtualForceGeneration(Vector3d& q, Vector3d& car_position)
{
    // 机械臂正运动学计算
    Ps(0,0)=(l1*cos(q(1,0))+l2*cos(q(1,0)+q(2,0)))*cos(q(0,0));
    Ps(1,0)=(l1*cos(q(1,0))+l2*cos(q(1,0)+q(2,0)))*sin(q(0,0));
    Ps(2,0)=(l1*sin(q(1,0))+l2*sin(q(1,0)+q(2,0)));
    Vector3d delta_position; // 双目相机相对于机械臂基座的位置偏差
    delta_position(0,0) = 0.2;
    delta_position(1,0) = 0;
    delta_position(2,0) = 0.2;
    car_position2base(0,0) = car_position(2,0) - delta_position(0,0);
    car_position2base(1,0) = car_position(0,0) - delta_position(1,0);
    car_position2base(2,0) = car_position(1,0) - delta_position(2,0);

    // 生成力反馈曲线
    ForceFeedback(0,0) = 3*atan((car_position2base(0,0)-Ps(0,0))/sqrt((car_position2base(0,0)-Ps(0,0))*(car_position2base(0,0)-Ps(0,0))));
    ForceFeedback(1,0) = 3*atan((car_position2base(1,0)-Ps(1,0))/sqrt((car_position2base(1,0)-Ps(1,0))*(car_position2base(1,0)-Ps(1,0))));
    ForceFeedback(2,0) = 3*atan((car_position2base(2,0)-Ps(2,0))/sqrt((car_position2base(2,0)-Ps(2,0))*(car_position2base(2,0)-Ps(2,0))));

    return ForceFeedback;

}
// 遥操作回调函数
bool InitializeOnlinePlannerFromReference()
{
    if (!trajectory_planner_enabled || !online_joint_planner)
    {
        return false;
    }
    const JointVector position {{expect_q(0,0), expect_q(1,0), expect_q(2,0), joint4_actual_angle}};
    const JointVector velocity {{expect_dq(0,0), expect_dq(1,0), expect_dq(2,0), joint4_actual_velocity}};
    const JointVector acceleration {{expect_ddq(0,0), expect_ddq(1,0), expect_ddq(2,0), 0.0}};
    std::string error;
    if (!online_joint_planner->initialize(position, velocity, acceleration, &error))
    {
        ROS_ERROR("Failed to initialize online trajectory planner: %s", error.c_str());
        planner_result = "error_initialization";
        return false;
    }
    raw_target_q = position;
    raw_target_dq = velocity;
    raw_target_ddq = acceleration;
    joint4angle = position[3];
    expect_q4_velocity = velocity[3];
    expect_q4_acceleration = acceleration[3];
    online_joint_planner_initialized = true;
    planner_result = "initialized";
    return true;
}

void RecordMotionLogSample(std::uint64_t loop_index, double dt_sec)
{
    if (!motion_data_log_enabled || !motion_csv_logger || !motion_csv_logger->running())
    {
        return;
    }

    mainpulator_motion::MotionLogSample sample;
    sample.wall_time_ns = ros::WallTime::now().toNSec();
    sample.ros_time_ns = ros::Time::now().toNSec();
    sample.loop_index = loop_index;
    sample.dt_sec = dt_sec;
    sample.active_source = active_control_source;
    sample.command_ind = latest_command_ind;
    sample.planner_enabled = trajectory_planner_enabled && online_joint_planner_initialized &&
                             !startup_homing_active;
    sample.planner_result = startup_homing_active ? "startup_homing" : planner_result;

    JointVector logged_raw_q = raw_target_q;
    JointVector logged_raw_dq = raw_target_dq;
    JointVector logged_raw_ddq = raw_target_ddq;
    if (startup_homing_active)
    {
        logged_raw_q = JointVector {{expect_q(0,0), expect_q(1,0), expect_q(2,0), joint4_actual_angle}};
        logged_raw_dq = JointVector {{expect_dq(0,0), expect_dq(1,0), expect_dq(2,0), 0.0}};
        logged_raw_ddq = JointVector {{expect_ddq(0,0), expect_ddq(1,0), expect_ddq(2,0), 0.0}};
    }
    const JointVector expected_position {{expect_q(0,0), expect_q(1,0), expect_q(2,0), joint4angle}};
    const JointVector expected_velocity {{expect_dq(0,0), expect_dq(1,0), expect_dq(2,0), expect_q4_velocity}};
    const JointVector expected_acceleration {{expect_ddq(0,0), expect_ddq(1,0), expect_ddq(2,0), expect_q4_acceleration}};
    const JointVector actual_position {{q(0,0), q(1,0), q(2,0), joint4_actual_angle}};
    const JointVector actual_velocity {{dq(0,0), dq(1,0), dq(2,0), joint4_actual_velocity}};
    for (std::size_t joint = 0; joint < 4; ++joint)
    {
        const double sign = joint == 0 ? -1.0 : 1.0;
        sample.raw_q[joint] = sign * logged_raw_q[joint];
        sample.raw_dq[joint] = sign * logged_raw_dq[joint];
        sample.raw_ddq[joint] = sign * logged_raw_ddq[joint];
        sample.expected_q[joint] = sign * expected_position[joint];
        sample.expected_dq[joint] = sign * expected_velocity[joint];
        sample.expected_ddq[joint] = sign * expected_acceleration[joint];
        sample.actual_q[joint] = sign * actual_position[joint];
        sample.actual_dq[joint] = sign * actual_velocity[joint];
    }
    motion_csv_logger->push(sample);
}

void PublishMotionStatus(const std::string& state, double progress, uint32_t ind, const std::string& source)
{
    if (!motion_status_pub) return;
    std_msgs::String message;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6)
           << "{\"ind\":" << ind
           << ",\"source\":\"" << source
           << "\",\"state\":\"" << state
           << "\",\"progress\":" << std::max(0.0, std::min(1.0, progress))
           << ",\"duration_sec\":" << runtime_motion_report_duration
           << ",\"stamp\":" << ros::Time::now().toSec() << "}";
    message.data = stream.str();
    motion_status_pub.publish(message);
}

void StartRuntimeMotion(const sensor_msgs::Imu& msg, const std::string& source)
{
    if (runtime_motion_id_valid && last_runtime_motion_ind == msg.header.seq &&
        last_runtime_motion_source == source && last_runtime_motion_stamp == msg.header.stamp)
    {
        return;
    }
    if (trajectory_planner_enabled)
    {
        if (!ApplyLogicalArmCommand(msg, source))
        {
            PublishMotionStatus("failed", 0.0, msg.header.seq, source);
            return;
        }
        if (runtime_motion.active)
        {
            PublishMotionStatus("preempted",
                                online_joint_planner ? online_joint_planner->progress() : 0.0,
                                runtime_motion.ind, runtime_motion.source);
        }
        runtime_motion.active = true;
        runtime_motion.ind = msg.header.seq;
        runtime_motion.source = source;
        runtime_motion.start_time = ros::WallTime::now();
        runtime_motion.last_status_progress = -1.0;
        runtime_motion_report_duration = 0.0;
        runtime_motion_id_valid = true;
        last_runtime_motion_ind = msg.header.seq;
        last_runtime_motion_source = source;
        last_runtime_motion_stamp = msg.header.stamp;
        latest_command_ind = msg.header.seq;
        latest_command_source = source;
        if (private_node_handle != NULL) private_node_handle->setParam("runtime_motion_complete", false);
        PublishMotionStatus("accepted", 0.0, runtime_motion.ind, runtime_motion.source);
        return;
    }
    if (runtime_motion.active)
    {
        const double elapsed = (ros::WallTime::now() - runtime_motion.start_time).toSec();
        PublishMotionStatus("preempted", elapsed / runtime_motion_duration,
                            runtime_motion.ind, runtime_motion.source);
    }

    const Vector3d start_q = expect_q;
    const Vector3d start_dq = expect_dq;
    const Vector3d start_ddq = expect_ddq;
    if (!ApplyLogicalArmCommand(msg, source))
    {
        PublishMotionStatus("failed", 0.0, msg.header.seq, source);
        return;
    }
    const Vector3d target_q = expect_q;
    expect_q = start_q;
    expect_dq = start_dq;
    expect_ddq = start_ddq;

    const double duration = runtime_motion_duration;
    const double duration2 = duration * duration;
    const double duration3 = duration2 * duration;
    const double duration4 = duration3 * duration;
    const double duration5 = duration4 * duration;
    const Vector3d delta = target_q - start_q;
    runtime_motion.coefficient[0] = start_q;
    runtime_motion.coefficient[1] = start_dq;
    runtime_motion.coefficient[2] = 0.5 * start_ddq;
    runtime_motion.coefficient[3] =
        (20.0 * delta - 12.0 * start_dq * duration - 3.0 * start_ddq * duration2) /
        (2.0 * duration3);
    runtime_motion.coefficient[4] =
        (-30.0 * delta + 16.0 * start_dq * duration + 3.0 * start_ddq * duration2) /
        (2.0 * duration4);
    runtime_motion.coefficient[5] =
        (12.0 * delta - 6.0 * start_dq * duration - start_ddq * duration2) /
        (2.0 * duration5);
    runtime_motion.active = true;
    runtime_motion.ind = msg.header.seq;
    runtime_motion.source = source;
    runtime_motion.start_time = ros::WallTime::now();
    runtime_motion.last_status_progress = -1.0;
    runtime_motion_report_duration = runtime_motion_duration;
    runtime_motion_id_valid = true;
    last_runtime_motion_ind = msg.header.seq;
    last_runtime_motion_source = source;
    last_runtime_motion_stamp = msg.header.stamp;
    latest_command_ind = msg.header.seq;
    latest_command_source = source;
    if (private_node_handle != NULL) private_node_handle->setParam("runtime_motion_complete", false);
    PublishMotionStatus("accepted", 0.0, runtime_motion.ind, runtime_motion.source);
}

void UpdateRuntimeMotion()
{
    if (trajectory_planner_enabled)
    {
        if (!online_joint_planner_initialized && !InitializeOnlinePlannerFromReference())
        {
            expect_dq.setZero();
            expect_ddq.setZero();
            expect_q4_velocity = 0.0;
            expect_q4_acceleration = 0.0;
            if (runtime_motion.active)
            {
                runtime_motion.active = false;
                if (private_node_handle != NULL) private_node_handle->setParam("runtime_motion_complete", true);
                PublishMotionStatus("failed", 0.0, runtime_motion.ind, runtime_motion.source);
            }
            else
            {
                PublishMotionStatus("failed", 0.0, latest_command_ind, latest_command_source);
            }
            return;
        }

        const ruckig::Result result = online_joint_planner->update();
        planner_result = mainpulator_motion::OnlineJointPlanner::resultName(result);
        if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
        {
            const JointVector& planned_q = online_joint_planner->position();
            const JointVector& planned_dq = online_joint_planner->velocity();
            const JointVector& planned_ddq = online_joint_planner->acceleration();
            for (int joint = 0; joint < 3; ++joint)
            {
                expect_q(joint,0) = planned_q[static_cast<std::size_t>(joint)];
                expect_dq(joint,0) = planned_dq[static_cast<std::size_t>(joint)];
                expect_ddq(joint,0) = planned_ddq[static_cast<std::size_t>(joint)];
            }
            joint4angle = planned_q[3];
            expect_q4_velocity = planned_dq[3];
            expect_q4_acceleration = planned_ddq[3];
            runtime_motion_report_duration = online_joint_planner->trajectoryDuration();

            if (runtime_motion.active)
            {
                const double progress = online_joint_planner->progress();
                if (runtime_motion.last_status_progress < 0.0 ||
                    progress - runtime_motion.last_status_progress >= 0.02)
                {
                    PublishMotionStatus("running", progress, runtime_motion.ind, runtime_motion.source);
                    runtime_motion.last_status_progress = progress;
                }
                if (result == ruckig::Result::Finished)
                {
                    runtime_motion.active = false;
                    if (private_node_handle != NULL) private_node_handle->setParam("runtime_motion_complete", true);
                    PublishMotionStatus("complete", 1.0, runtime_motion.ind, runtime_motion.source);
                }
            }
            return;
        }

        const JointVector& held_q = online_joint_planner->position();
        expect_q(0,0) = held_q[0];
        expect_q(1,0) = held_q[1];
        expect_q(2,0) = held_q[2];
        joint4angle = held_q[3];
        expect_dq.setZero();
        expect_ddq.setZero();
        expect_q4_velocity = 0.0;
        expect_q4_acceleration = 0.0;
        const double failure_progress = online_joint_planner->progress();
        online_joint_planner->hold();
        ROS_ERROR_THROTTLE(1.0, "Online trajectory planner failed: %s", planner_result.c_str());
        if (runtime_motion.active)
        {
            runtime_motion.active = false;
            if (private_node_handle != NULL) private_node_handle->setParam("runtime_motion_complete", true);
            PublishMotionStatus("failed", failure_progress,
                                runtime_motion.ind, runtime_motion.source);
        }
        else
        {
            PublishMotionStatus("failed", failure_progress,
                                latest_command_ind, latest_command_source);
        }
        return;
    }

    if (!runtime_motion.active) return;
    const double elapsed = (ros::WallTime::now() - runtime_motion.start_time).toSec();
    const double t = std::max(0.0, std::min(runtime_motion_duration, elapsed));
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    expect_q = runtime_motion.coefficient[0] + runtime_motion.coefficient[1] * t +
               runtime_motion.coefficient[2] * t2 + runtime_motion.coefficient[3] * t3 +
               runtime_motion.coefficient[4] * t4 + runtime_motion.coefficient[5] * t5;
    expect_dq = runtime_motion.coefficient[1] + 2.0 * runtime_motion.coefficient[2] * t +
                3.0 * runtime_motion.coefficient[3] * t2 + 4.0 * runtime_motion.coefficient[4] * t3 +
                5.0 * runtime_motion.coefficient[5] * t4;
    expect_ddq = 2.0 * runtime_motion.coefficient[2] + 6.0 * runtime_motion.coefficient[3] * t +
                 12.0 * runtime_motion.coefficient[4] * t2 + 20.0 * runtime_motion.coefficient[5] * t3;

    const double progress = t / runtime_motion_duration;
    if (runtime_motion.last_status_progress < 0.0 || progress - runtime_motion.last_status_progress >= 0.02)
    {
        PublishMotionStatus("running", progress, runtime_motion.ind, runtime_motion.source);
        runtime_motion.last_status_progress = progress;
    }
    if (elapsed >= runtime_motion_duration)
    {
        expect_dq.setZero();
        expect_ddq.setZero();
        runtime_motion.active = false;
        if (private_node_handle != NULL) private_node_handle->setParam("runtime_motion_complete", true);
        PublishMotionStatus("complete", 1.0, runtime_motion.ind, runtime_motion.source);
    }
}

void CancelRuntimeMotion(const std::string& state)
{
    if (trajectory_planner_enabled)
    {
        if (runtime_motion.active)
        {
            PublishMotionStatus(state,
                                online_joint_planner ? online_joint_planner->progress() : 0.0,
                                runtime_motion.ind, runtime_motion.source);
        }
        runtime_motion.active = false;
        expect_q = q;
        expect_dq.setZero();
        expect_ddq.setZero();
        joint4angle = joint4_actual_angle;
        expect_q4_velocity = 0.0;
        expect_q4_acceleration = 0.0;
        raw_target_q = JointVector {{q(0,0), q(1,0), q(2,0), joint4_actual_angle}};
        raw_target_dq.fill(0.0);
        raw_target_ddq.fill(0.0);
        KB_D = 0.0;
        planner_result = "estop_hold";
        if (online_joint_planner)
        {
            std::string error;
            online_joint_planner_initialized = online_joint_planner->initialize(
                raw_target_q, raw_target_dq, raw_target_ddq, &error);
            if (!online_joint_planner_initialized)
            {
                planner_result = "error_estop_hold";
                ROS_ERROR("Failed to reset planner during stop: %s", error.c_str());
            }
        }
        if (private_node_handle != NULL) private_node_handle->setParam("runtime_motion_complete", true);
        return;
    }
    if (!runtime_motion.active) return;
    const double elapsed = (ros::WallTime::now() - runtime_motion.start_time).toSec();
    PublishMotionStatus(state, elapsed / runtime_motion_duration,
                        runtime_motion.ind, runtime_motion.source);
    runtime_motion.active = false;
    expect_q = q;
    expect_dq.setZero();
    expect_ddq.setZero();
    joint4angle = joint4_actual_angle;
    KB_D = 0.0;
    if (private_node_handle != NULL) private_node_handle->setParam("runtime_motion_complete", true);
}

void ActiveControlSourceCallback(const std_msgs::String& msg)
{
    static const std::vector<std::string> allowed = {
        "idle", "keyboard", "gamepad", "controller_delta", "hand_vision",
        "teleop", "imitation", "home", "preset", "estop"
    };
    if (std::find(allowed.begin(), allowed.end(), msg.data) == allowed.end())
    {
        ROS_WARN("Rejected unknown arm control source: %s", msg.data.c_str());
        return;
    }
    if (active_control_source != msg.data)
    {
        if (runtime_motion.active && msg.data != "home" && msg.data != "preset" && msg.data != "estop")
        {
            ROS_WARN_THROTTLE(1.0, "Ignoring control source change during runtime motion");
            return;
        }
        if (msg.data == "estop")
        {
            CancelRuntimeMotion("preempted");
        }
        // ROS_INFO("Arm control source: %s -> %s", active_control_source.c_str(), msg.data.c_str());
        active_control_source = msg.data;
    }
}

void TeleOperationCallback(const sensor_msgs::Imu& msg)
{
    if (startup_homing_active)
    {
        ROS_WARN_THROTTLE(1.0, "Ignoring teleoperation command during torque startup homing");
        return;
    }
    if (active_control_source != "teleop") return;
    ApplyLogicalArmCommand(msg, "teleop");
}

void H5TeleOperationCallback(const sensor_msgs::Imu& msg)
{
    if (startup_homing_active)
    {
        ROS_WARN_THROTTLE(1.0, "Ignoring H5 command during torque startup homing");
        return;
    }
    const std::string prefix = "h5:";
    const std::string frame_id = msg.header.frame_id;
    if (frame_id.compare(0, prefix.size(), prefix) != 0) return;
    const std::string command_source = frame_id.substr(prefix.size());
    const bool runtime_source = command_source == "home" || command_source == "preset";
    if (command_source != active_control_source && !runtime_source) return;
    if (runtime_source)
    {
        active_control_source = command_source;
        StartRuntimeMotion(msg, command_source);
        return;
    }
    if (runtime_motion.active) return;
    if (trajectory_planner_enabled && command_source == "estop")
    {
        CancelRuntimeMotion("preempted");
        return;
    }
    ApplyLogicalArmCommand(msg, command_source);
}

bool ApplyLogicalArmCommand(const sensor_msgs::Imu& msg, const std::string& source)
{
    JointVector target_q {{-msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w}};
    JointVector target_dq {{-msg.angular_velocity.x, msg.angular_velocity.y,
                            msg.angular_velocity.z, 0.0}};
    JointVector target_ddq {{-msg.linear_acceleration.x, msg.linear_acceleration.y,
                              msg.linear_acceleration.z, 0.0}};
    if (trajectory_planner_enabled && source != "teleop")
    {
        target_dq.fill(0.0);
        target_ddq.fill(0.0);
    }
    const bool finite_target =
        std::all_of(target_q.begin(), target_q.end(), [](double value) { return std::isfinite(value); }) &&
        std::all_of(target_dq.begin(), target_dq.end(), [](double value) { return std::isfinite(value); }) &&
        std::all_of(target_ddq.begin(), target_ddq.end(), [](double value) { return std::isfinite(value); });
    if (!finite_target)
    {
        ROS_WARN("Rejected non-finite arm target from source '%s'", source.c_str());
        return false;
    }

    if (trajectory_planner_enabled)
    {
        if (!online_joint_planner_initialized && !InitializeOnlinePlannerFromReference())
        {
            return false;
        }
        std::string planner_error;
        if (!online_joint_planner->setTarget(target_q, target_dq, target_ddq, &planner_error))
        {
            ROS_WARN("Rejected trajectory target from source '%s': %s",
                     source.c_str(), planner_error.c_str());
            return false;
        }
    }
    else
    {
        expect_q(0,0) = target_q[0];
        expect_q(1,0) = target_q[1];
        expect_q(2,0) = target_q[2];
        expect_dq(0,0) = target_dq[0];
        expect_dq(1,0) = target_dq[1];
        expect_dq(2,0) = target_dq[2];
        expect_ddq(0,0) = target_ddq[0];
        expect_ddq(1,0) = target_ddq[1];
        expect_ddq(2,0) = target_ddq[2];
        joint4angle = target_q[3];
        expect_q4_velocity = 0.0;
        expect_q4_acceleration = 0.0;
        planner_result = "disabled";
    }
    raw_target_q = target_q;
    raw_target_dq = target_dq;
    raw_target_ddq = target_ddq;
    latest_command_ind = msg.header.seq;
    latest_command_source = source;

    // ROS_INFO("expect_q1=%lf",expect_q(0,0));
    // ROS_INFO("expect_q2=%lf",expect_q(1,0));
    // ROS_INFO("expect_q3=%lf",expect_q(2,0));
    // ROS_INFO("joint4angle=%lf",joint4angle);

    expect_dq(0,0) = -msg.angular_velocity.x;
    expect_dq(1,0) = msg.angular_velocity.y;
    expect_dq(2,0) = msg.angular_velocity.z;

    expect_ddq(0,0) = -msg.linear_acceleration.x;
    expect_ddq(1,0) = msg.linear_acceleration.y;
    expect_ddq(2,0) = msg.linear_acceleration.z;

    // H5 hand-vision gripper command, forwarded by h5_udp_bridge from order[10].
    //  1: close, 0: hold/no move, -1: open.
    KB_D = msg.orientation_covariance[0];
    if (KB_D > 1.0)
    {
        KB_D = 1.0;
    }
    else if (KB_D < -1.0)
    {
        KB_D = -1.0;
    }
    if (KB_D > -0.01 && KB_D < 0.01)
    {
        KB_D = 0.0;
    }
    return true;
}
// 【修改代码】接收模仿学习节点下发的期望轨迹的回调函数（已集成抓手信号）
void ImitationCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (startup_homing_active)
    {
        ROS_WARN_THROTTLE(1.0, "Ignoring imitation command during torque startup homing");
        return;
    }
    if (active_control_source != "imitation") return;
    // 【修改】检查接收到的数据维度是否正确，position现在需要4个元素
    if (msg->position.size() < 4 || msg->velocity.size() < 3 || msg->effort.size() < 3)
    {
        ROS_WARN("Received imitation trajectory message with incorrect dimensions. Position array should have 4 elements (q0,q1,q2,q4).");
        return;
    }

    const JointVector target_q {{-msg->position[0], msg->position[1], msg->position[2], msg->position[3]}};
    const JointVector target_dq {{-msg->velocity[0], msg->velocity[1], msg->velocity[2], 0.0}};
    const JointVector target_ddq {{-msg->effort[0], msg->effort[1], msg->effort[2], 0.0}};
    const bool finite_target =
        std::all_of(target_q.begin(), target_q.end(), [](double value) { return std::isfinite(value); }) &&
        std::all_of(target_dq.begin(), target_dq.end(), [](double value) { return std::isfinite(value); }) &&
        std::all_of(target_ddq.begin(), target_ddq.end(), [](double value) { return std::isfinite(value); });
    if (!finite_target)
    {
        ROS_WARN("Rejected non-finite imitation trajectory target");
        return;
    }
    if (trajectory_planner_enabled)
    {
        if (!online_joint_planner_initialized && !InitializeOnlinePlannerFromReference())
        {
            return;
        }
        std::string planner_error;
        if (!online_joint_planner->setTarget(target_q, target_dq, target_ddq, &planner_error))
        {
            ROS_WARN("Rejected imitation trajectory target: %s", planner_error.c_str());
            return;
        }
    }
    else
    {
        expect_q(0,0) = target_q[0];
        expect_q(1,0) = target_q[1];
        expect_q(2,0) = target_q[2];
        expect_dq(0,0) = target_dq[0];
        expect_dq(1,0) = target_dq[1];
        expect_dq(2,0) = target_dq[2];
        expect_ddq(0,0) = target_ddq[0];
        expect_ddq(1,0) = target_ddq[1];
        expect_ddq(2,0) = target_ddq[2];
        joint4angle = target_q[3];
        expect_q4_velocity = 0.0;
        expect_q4_acceleration = 0.0;
        planner_result = "disabled";
    }
    raw_target_q = target_q;
    raw_target_dq = target_dq;
    raw_target_ddq = target_ddq;
    latest_command_ind = msg->header.seq;
    latest_command_source = "imitation";
}


void MainpulatorCallback(const can_msgs::Frame &receive_message)
{
        switch (receive_message.id)
        {
        case 0x181:
            joint1.current_angle(receive_message);
            q(0, 0) = joint1.get_current_angle();
            joint_position_received[0] = true;
            //ROS_INFO("Joint1 = %lf", q(0, 0));
            break;
        case 0x182:
            joint2.current_angle(receive_message);
            q(1, 0) = joint2.get_current_angle();
            joint_position_received[1] = true;
            //ROS_INFO("Joint1 = %lf", q(2, 0));
            break;
        case 0x183:
            joint3.current_angle(receive_message);
            q(2, 0) = joint3.get_current_angle();
            joint_position_received[2] = true;
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
            joint_velocity_received[0] = true;
            break;
        case 0x282:
            joint2.current_velocity(receive_message);
            dq(1, 0) = joint2.get_current_velocity();
            joint_velocity_received[1] = true;
            break;
        case 0x283:
            joint3.current_velocity(receive_message);
            dq(2, 0) = joint3.get_current_velocity();
            joint_velocity_received[2] = true;
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
    lambda1(0,0)  = 5;
    lambda1(1,1)  = 4;
    lambda1(2,2)  = 4.6;
    lambda2(0,0)  = 27;
    lambda2(1,1)  = 11;
    lambda2(2,2)  = 31;

    // 我的参数  
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

    gammamamm(0,0) = 500/10;
    gammamamm(1,1) = 100/10;

    gammamamm(2,2) = 1000/10;
    gammamamm(3,3) = 1200/10;
    gammamamm(4,4) = 1500/10;

    gammamamm(5,5) = 1500/10;
    gammamamm(6,6) = 1500/10;
    gammamamm(7,7) = 1500/10;

    gammamamm(8,8) = 1000/2.5;
    gammamamm(9,9) = 1000/2.5;
    gammamamm(10,10) = 1000/2.5;
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

   Vector3d ua = -1.0*fai*theta;
   Vector3d us = - z1 - lambda2*z2;
   //ROS_INFO("ua1 =%lf ua2 = %lf ua3=%lf ",ua(0,0),ua(1,0),ua(2,0));
   //ROS_INFO("us1 =%lf us2 = %lf us3=%lf ",us(0,0),us(1,0),us(2,0));
   
    // 调试用：
    // u = ua;  //先检查模型补偿项，应该占很大的部分,
    // u = us;  //再检查反馈项，应该占很小的部分
    u = ua + us;
    return u;  //输出tol_s
}
