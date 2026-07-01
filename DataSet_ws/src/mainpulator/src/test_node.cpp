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
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <algorithm>
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
string control_type;
string active_control_source = "idle";
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
double m1 = 4.8;
double m2  = 3.2;
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
void ApplyLogicalArmCommand(const sensor_msgs::Imu& msg);

void CarPosition_Callback(const  std_msgs::Float64MultiArray& msg);

// 【新增代码】模仿学习回调函数的前向声明
void ImitationCallback(const sensor_msgs::JointState::ConstPtr& msg);

Vector3d VirtualForceGeneration(Vector3d& q, Vector3d& car_position);

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
    // 2. 订阅来自 AI PC 的期望轨迹
    ros::Subscriber imitation_trajectory_sub = nh.subscribe<sensor_msgs::JointState>("/imitation/desired_trajectory", 10, ImitationCallback);


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


    ros::Rate loop_rate(20);
    int socket_can = param::SocketCANInit();
    ROS_INFO("socket_can = %d", socket_can);
    ROS_INFO("control_type  =  %s", control_type.c_str());

    can_msgs::Frame send_message;

    //机械臂输出化配置
    ROS_INFO("init");
    param::PositionInit(joint1, socket_can);
    param::PositionInit(joint2, socket_can);
    param::PositionInit(joint3, socket_can);
    //param::MomentInit(joint5, socket_can);
    //param::MomentInit(joint1, socket_can);
    //param::MomentInit(joint2, socket_can);
    //param::MomentInit(joint3, socket_can);
    param::PositionInit(joint4, socket_can);
    usleep(500000);
    //机械臂使能
    param::Enable(socket_can, 6, joint1);
    param::Enable(socket_can, 6, joint2);
    param::Enable(socket_can, 6, joint3);
    param::Enable(socket_can, 6, joint4);
    //param::Enable(socket_can, 6, joint5);
    usleep(500000);
    ROS_INFO("init end");

    AdaptiveBACKSTEPPINGparam_init();

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

    

        // 自适应反步控制（已注释，改为位置环控制）
        // tol=AdaptiveBackstepping(expect_q,expect_dq,expect_ddq,q,dq,zerovector,0.01);
        ROS_INFO("q_e=[%f,%f,%f,%f]", expect_q(0,0), expect_q(1,0), expect_q(2,0), KB_D);
        //ROS_INFO("dq_e=[%f,%f,%f]", expect_dq(0,0), expect_dq(1,0), expect_dq(2,0));
        //ROS_INFO("car_position = %lf, %lf, %lf", car_position(0,0), car_position(1,0), car_position(2,0));
        ROS_INFO("q = %lf, %lf, %lf", q(0,0), q(1,0), q(2,0));
       
       
/*
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
        //ROS_INFO("tol1 =%lf tol2 = %lf tol3=%lf ",tol(0,0),tol(1,0),tol(2,0));


        can_msgs::Frame frames;
        send_message = joint1.MomentOutput(tol(0,0));
        socketcan_send_pub.publish(send_message);

        send_message = joint2.MomentOutput(tol(1,0));
        socketcan_send_pub.publish(send_message);

        send_message = joint3.MomentOutput80(tol(2,0));
        socketcan_send_pub.publish(send_message);
*/

        // --- 改为纯位置环接收遥操作信息发指令 ---
        can_msgs::Frame frames;
        send_message = joint1.set_angle_for_new_joint(expect_q(0,0));
        socketcan_send_pub.publish(send_message);
        send_message =joint1.set_angle(expect_q(0,0));
        socketcan_send_pub.publish(send_message);

        send_message = joint2.set_angle_for_new_joint(expect_q(1,0));
        socketcan_send_pub.publish(send_message);
        send_message =joint2.set_angle(expect_q(1,0));
        socketcan_send_pub.publish(send_message);

        send_message = joint3.set_angle_for_new_joint(expect_q(2,0));
        socketcan_send_pub.publish(send_message);
        send_message =joint3.set_angle(expect_q(2,0));
        socketcan_send_pub.publish(send_message);
        // ----------------------------------------

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
        ROS_INFO("joint5_actual_angle=%lf",joint5_actual_angle);
        //ROS_INFO("gripper_flag=%lf",gripper_flag);
        send_message = joint5.MomentOutput(tol5);
        socketcan_send_pub.publish(send_message);



        frames.id = 0x80;
        frames.dlc = 0;
        socketcan_send_pub.publish(frames);
        loop_rate.sleep();
    }
}


// 接收到双目识别小车位置的回调函数
void CarPosition_Callback(const  std_msgs::Float64MultiArray& msg)
{
    car_position(0,0)=msg.data[0]/1000;  
    car_position(1,0)=msg.data[1]/1000;
    car_position(2,0)=msg.data[2]/1000;
    ROS_INFO("car_position = %lf, %lf, %lf", car_position(0,0), car_position(1,0), car_position(2,0));
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
        ROS_INFO("Arm control source: %s -> %s", active_control_source.c_str(), msg.data.c_str());
        active_control_source = msg.data;
    }
}

void TeleOperationCallback(const sensor_msgs::Imu& msg)
{
    if (active_control_source != "teleop") return;
    ApplyLogicalArmCommand(msg);
}

void H5TeleOperationCallback(const sensor_msgs::Imu& msg)
{
    const std::string prefix = "h5:";
    const std::string frame_id = msg.header.frame_id;
    if (frame_id.compare(0, prefix.size(), prefix) != 0) return;
    const std::string command_source = frame_id.substr(prefix.size());
    if (command_source != active_control_source) return;
    ApplyLogicalArmCommand(msg);
}

void ApplyLogicalArmCommand(const sensor_msgs::Imu& msg)
{
    expect_q(0,0) = -msg.orientation.x;
    expect_q(1,0) = msg.orientation.y;
    expect_q(2,0) = msg.orientation.z;
    joint4angle = msg.orientation.w;

    ROS_INFO("expect_q1=%lf",expect_q(0,0));
    ROS_INFO("expect_q2=%lf",expect_q(1,0));
    ROS_INFO("expect_q3=%lf",expect_q(2,0));
    ROS_INFO("joint4angle=%lf",joint4angle);

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
}
// 【修改代码】接收模仿学习节点下发的期望轨迹的回调函数（已集成抓手信号）
void ImitationCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (active_control_source != "imitation") return;
    // 【修改】检查接收到的数据维度是否正确，position现在需要4个元素
    if (msg->position.size() < 4 || msg->velocity.size() < 3 || msg->effort.size() < 3)
    {
        ROS_WARN("Received imitation trajectory message with incorrect dimensions. Position array should have 4 elements (q0,q1,q2,q4).");
        return;
    }

    // 解析消息，更新期望轨迹
    expect_q(0,0) = -msg->position[0];
    expect_q(1,0) = msg->position[1];
    expect_q(2,0) = msg->position[2];

    // position[3] is joint4 target angle. It is independent from J1-J3 IK.
    joint4angle = msg->position[3];

    // 解析期望角速度和角加速度（这部分不变）
    expect_dq(0,0) = -msg->velocity[0];
    expect_dq(1,0) = msg->velocity[1];
    expect_dq(2,0) = msg->velocity[2];

    expect_ddq(0,0) = msg->effort[0]; // 复用 effort 字段
    expect_ddq(1,0) = msg->effort[1];
    expect_ddq(2,0) = msg->effort[2];
    expect_ddq(0,0) = -expect_ddq(0,0);

    // 取消下面这行代码的注释以进行调试
    // ROS_INFO("Received new trajectory: q=[%f,%f,%f], q4=%f", expect_q(0,0), expect_q(1,0), expect_q(2,0), joint4angle);
}


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
    lambda1(0,0)  = 8;
    lambda1(1,1)  = 8;
    lambda1(2,2)  = 4; 

    lambda2(0,0)  = 30;
    lambda2(1,1)  = 30;
    lambda2(2,2)  = 20; 

    // 我的参数  
    theta(0,0) = 4;
    theta(1,0) = 4.5;

    
    theta(2,0) = 10;
    theta(3,0) = 10;
    theta(4,0) = 10;

    theta(5,0) = 7.651;
    theta(6,0) = 7.651;
    theta(7,0) = 7.994;

    theta(8,0) = 0;
    theta(9,0) = 0;
    theta(10,0) = 0;
    
    /*
    theta(2,0) = 0;
    theta(3,0) = 0;
    theta(4,0) = 0;

    theta(5,0) = 0;
    theta(6,0) = 0;
    theta(7,0) = 0;

    theta(8,0) = 0;
    theta(9,0) = -30;
    theta(10,0) = -5;
    */

    theta_min(0,0) = 3;
    theta_max(0,0) = 6;
    theta_min(1,0) = 4;
    theta_max(1,0) = 5;
    
    theta_min(2,0) = 5;
    theta_max(2,0) = 15;
    theta_max(3,0) = 15;
    theta_min(3,0) = 5;
    theta_max(4,0) = 15;
    theta_min(4,0) = 5;
    
    theta_max(5,0) = 15;
    theta_min(5,0) = 5;
    theta_max(6,0) = 15;
    theta_min(6,0) = 5;
    theta_max(7,0) = 15;
    theta_min(7,0) = 5;
    
    theta_max(8,0) = 30;
    theta_min(8,0) =-30;
    theta_max(9,0) = 50;
    theta_min(9,0) =-50;
    theta_max(10,0) = 30;
    theta_min(10,0) =-30;

    /*
    theta_min(0,0) = 0;
    theta_max(0,0) = 0;
    theta_min(1,0) = 0;
    theta_max(1,0) = 0;
    
    theta_min(2,0) = 0;
    theta_max(2,0) = 0;
    theta_max(3,0) = 0;
    theta_min(3,0) = 0;
    theta_max(4,0) = 0;
    theta_min(4,0) = 0;
    
    theta_max(5,0) = 0;
    theta_min(5,0) = 0;
    theta_max(6,0) = 0;
    theta_min(6,0) = 0;
    theta_max(7,0) = 0;
    theta_min(7,0) = 0;
    */

    gammamamm(0,0) = 30;//300;
    gammamamm(1,1) = 10;//100;

    gammamamm(2,2) = 100;//1500;
    gammamamm(3,3) = 100;//1200;
    gammamamm(4,4) = 100;//1000;

    gammamamm(5,5) = 100;//1000;
    gammamamm(6,6) = 100;//1000;
    gammamamm(7,7) = 100;//1000;

    gammamamm(8,8) = 400;
    gammamamm(9,9) = 500;
    gammamamm(10,10) = 250;
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
    double deadzone = 0.005;

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
    fai(0,5) = (abs(dq(0,0)) < deadzone) ? 0 : -atan2(100*dq(0,0),1)*2.0/M_PI;;
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
    fai(1,6) = (abs(dq(1,0)) < deadzone) ? 0 : -atan2(100*dq(1,0),1)*2.0/M_PI; 
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
    fai(2,7) = (abs(dq(2,0)) < deadzone) ? 0 : -atan2(100*dq(2,0),1)*2.0/M_PI;
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
