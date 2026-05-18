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
string control_type ;
control::mainpulator joint1(1,262144);//pi   //12509
control::mainpulator joint2(2,262144);//pi   
control::mainpulator joint3(3,196608);//196608//0.75pi   //236009//367081//新机械臂327680向下，65536向上

control::mainpulator joint4(4,262144);
control::mainpulator joint5(5,2000);
std_msgs::Bool stop_flag ;
double expect_q1 = 0,expect_q2 = 0,expect_q3 = 0,enpect_q5 = -2;
double expect_dq1 = 0,expect_dq2 = 0,expect_dq3 = 0;
double expect_ddq1 = 0,expect_ddq2 = 0,expect_ddq3 = 0;
bool joint5_state = false;
//bool joint5_open = false;

bool joint5_open =false;
bool joint5_homing =false;
double  joint5_catch_TeleOpe=0.0;

int count_time = 0;
int count1 = 0;
int count_time_teleope = 0;
double tol3  = 0;
int  trajectory_flag = 0;
Vector3d Kinematics_Solver(double q1,double q2,double q3,double KB_W,double KB_A,double KB_Q,double KB_SPEEDUP,double SPEEDUP1,double SPEEDUP2,double SPEEDUP3);
Vector3d Kinematics_theta_update;
Vector3d Kinematics_expect_q;

Vector3d arc(Vector3d& expect_q, Vector3d& expect_qd, Vector3d& expect_qdd,Vector3d& q,Vector3d& qd,double t);
Vector3d slidemodecontrol(Vector3d& expect_q, Vector3d& expect_qd, Vector3d& expect_qdd,Vector3d& q,Vector3d& qd,double t);
void  param_init();

/********************************************/
    //键盘控制相关增量变量
    double q_TeleopeKB=0.00001;
    double KB_W=0.0;
    double KB_S=0.0;
    double KB_A=0.0;
    double KB_D=0.0;
    double KB_Q=0.0;
    double KB_E=0.0;
    double Aceleration_KB=0.0;
    double Velocity_KB=0.0;

    double joint4angle=0.0;
    double joint5angle=0.0;

    double gripper_ultimate = 6.2;

    double joint4_actual_angle=0.0;
    double joint5_actual_angle=0.0;
    double joint4_actual_velocity=0.0;
    double joint5_actual_current=0.0;

    double Kinematics_r=0.0;
    double Kinematics_x=0.0;
    double Kinematics_y=0.0;
    double Kinematics_z=0.0;


    double KB_SPEEDUP=0.0;
    double KB_TURNLR=0.0;
/********************************************/
    //imu相关变量
    double imu_R=0.00001;
    double imu_P=0.00001;
    double imu_Y=0.00001;

/********************************************/
double dthp1 = 0;
double dthp2 = 0;

double dthp3 = 0;
double dthp4 = 0;
double thp1 =15.5;
double thp2 = 13;
double thp3 = 15;
double thp4 = 0;

double us1_1 = 0;
double ua_1 = 0;
double us3 = 0;
double u_1 = 0;

double q1 = 0.001;
double q2 = -1.57;
double q3 = -1.57;
double gamma1 =200;
double gamma2 = 100;
double gamma3 = 100; 
double gamma4 = 500;
/************************************************/
/********************矩阵定义*********************/
double m1 = 10;
double m2  = 1.7;
double l1 = 0.424;
//double l2 = 0.268;
double l2 = 0.325;
//g根据安装方向修改 橙色底座为-------
double g = -9.8;
//实际角度
Vector3d q;
//实际角速度
Vector3d qd;
//期望角度
Vector3d expect_q;
//期望角速度
Vector3d  expect_qd;
//期望角加速度
Vector3d  expect_qdd;
Vector3d err ;
Vector3d derr ;

Vector2d M;

//参数表
Matrix3d k1;
Matrix3d k2;

Vector3d qr;
Vector3d dqr;
Vector3d qr_dot;
Vector3d dqr_dot;

VectorXd theta_d(11);
VectorXd theta(11);
VectorXd theta_max(11);
VectorXd theta_min(11);
VectorXd slidetheta(11);

Vector3d tol;

Vector3d sgns;
Vector3d us1 ;
Vector3d ua;
Vector3d u;
//Eigen::Matrix<double, 8, 8> gammamamm;  

MatrixXd gammamamm(11,11);
//Eigen::Matrix<double, 2, 8> fai_Te;  
MatrixXd fai(3,11);
//Eigen::Matrix<double, 2, 8> fai_T;  
MatrixXd fai_dot(3,11);
Matrix3Xd  theta_max1 (3,3);
Matrix3Xd Xite (3,3);

///240704
//来自qgc上位机的数据变量保存
double RE_q1=0.0;
double RE_q2=0.0;
double RE_q3=0.0;
double RE_q4=0.0;
double RE_q5 = 0.0;
double RE_qd1 = 0.0;
double RE_qd2 = 0.0;
double RE_qd3 = 0.0;
double RE_qd4 = 0.0;
double RE_Current5= 0.0;

double RE_KB_W=0.0;
double RE_KB_A=0.0;
double RE_KB_S=0.0;
double RE_KB_D=0.0;
double RE_KB_Q=0.0;
double RE_KB_E=0.0;

double RE_KB_L=0.0;
double RE_KB_R=0.0;
double RE_KB_C=0.0;
double RE_KB_O=0.0;

double RE_status=0.0;
bool RE_status_changed = false;
double RE_Buttom=0.0;
double PU_status=0.0;

double RE_status_lock_st0=0;
double RE_status_lock_st1=0;
double RE_status_lock_st2=0;
double RE_status_lock_st7=0;
double Status3_lock=0;
double Status4_lock=0;
double RE_Flag1=0.0;
double RE_Flag2=0.0;
double RE_Flag3=0.0;

int INTTT_q1=0;
int INTTT_q2=0;
int INTTT_q3=0;
int INTTT_q4=0;
int INTTT_q5=0;
void sotp_falg(const std_msgs::Bool& flag)
{
    //stop_flag.data = flag.data;
  //  ROS_INFO("stop");
}
//收到cameranode 话题发来的数据
void Mainpulator_ser(const sensor_msgs::JointState& joint_state)
{
    //expect_q = joint_state.position[0];
    //expect_qd = joint_state.velocity[0];
    //expect_qdd = joint_state.effort[0];
    //此处需要更改,为expectq读取cameranode pub回来的9+1个数据
    expect_q(0,0) = joint_state.position[1]/180*3.14;
    expect_q(1,0)  = joint_state.position[2]/180*3.14;
    expect_q(2,0)  = joint_state.position[3]/180*3.14;
    
    //expect_qd(0,0)  = joint_state.velocity[0]/180*3.14;
    //expect_qd(1,0) = joint_state.velocity[1]/180*3.14;
    //expect_qd(2,0) = joint_state.velocity[2]/180*3.14;

    //expect_qdd(0,0) = joint_state.effort[0]/180*3.14;
    //expect_qdd(1,0) = joint_state.effort[1]/180*3.14;
    //expect_qdd(2,0) = joint_state.effort[2]/180*3.14;

    //收到cameranode发来的第四位为int=1,即为可以进行抓取操作
    if(abs(joint_state.position[3]-1.0)<0.1)
    {
        //joint5_open = true;
        joint5_open =true;
    }
      //ROS_INFO("joint5  state  %lf",joint_state.position[4]);
    if(abs(joint_state.position[3]-2.0)<0.1)
    {
        //joint5_open = true;
        joint5_homing =true;
    }
  
}
/** 
 * @brief 位置模式回调函数,编码器返回速度、位置等信息
 * @param receive_message ros_canopen类对象
 * @return 
 */
void MainpulatorCallback(const can_msgs::Frame& receive_message);
/** 
 * @brief 位置模式回调函数,编码器返回速度、位置等信息
 * @param receive_message ros_canopen类对象
 * @return 
 */
void trajectoryCallback(const sensor_msgs::JointState& joint_state);

void TeleOperationCallback(const  sensor_msgs::Imu& msg);

void TeleopeKB_Callback(const geometry_msgs::Twist & msg);

void TeleopeJOY_Callback(const geometry_msgs::Twist & msg);

void Imu_Info_Callback(const sensor_msgs::JointState & msg);

void QGC_Callback(const sensor_msgs::JointState& msg);


void stopcallback(const std_msgs::Bool& flag){
        //joint5_open = true;
        joint5_open = true;
        ROS_INFO("joint5 open begin");
};

double arc_control(double expect_q,double expect_dq,double expect_ddq,double q,double dq,double t ,double m,double c,double g);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"mainpulator_param_node");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle nh("~");
    ros::NodeHandle socketcan_send;
    ros::NodeHandle socketcan_receive;
    ros::NodeHandle receive_stop;
    ros::NodeHandle trajectory_receive;
    ros::NodeHandle mainpulator_receive_sub;
    ros::NodeHandle mainpul_ser;
    ros::Subscriber socketcan_receive_sub;
    ros::Subscriber trajectory_receive_sub;

    stop_flag.data = false;
    //机械臂控制方式 位置控制 力矩控制
    nh.getParam("control_type",control_type);
    //控制器发布数据至机械臂
    ROS_INFO("control_type = %s",control_type.c_str());
    ros::Publisher socketcan_send_pub =  socketcan_send.advertise<can_msgs::Frame>("sent_messages", 10);
    ros::Subscriber receive_stop_sub = socketcan_receive.subscribe("reboot_stop",10,stopcallback);
    //cameranode发来的话题
    ros::Subscriber tarck_sub1 = mainpul_ser.subscribe("servermessage",100,Mainpulator_ser);
    //有问题
    ros::Subscriber mainpulator_stop_sub = mainpulator_receive_sub.subscribe("mainpulator_stop",100,sotp_falg);
     //遥操作，话题接收q qd qdd
    ros::NodeHandle TeleOperation;
    ros::Subscriber TeleOperation_sub = TeleOperation.subscribe("/pub_joint_state",10,TeleOperationCallback);

    //键盘操作
    ros::NodeHandle TeleopeKB;
    ros::Subscriber TeleopeKB_sub=TeleopeKB.subscribe("Cmd_KeyboardTeleop",1,TeleopeKB_Callback);
    //手柄操作
    ros::NodeHandle TeleopeJOY;
    ros::Subscriber TeleopeJOY_sub=TeleopeJOY.subscribe("joynode_transfer_pub",1,TeleopeJOY_Callback);
    //imu信息
    ros::NodeHandle Imu_Info;
    ros::Subscriber Imu_Info_sub=Imu_Info.subscribe("Imu_Info_",1,Imu_Info_Callback);
     //qgc上位机
    ros::NodeHandle QGC_Info;
    ros::Subscriber QGC_Info_sub = QGC_Info.subscribe("MVLink_Pub_To_Main_Node",10,QGC_Callback);
    expect_q(0,0)=0.00001;
    expect_q(1,0)=0.00001;
    expect_q(2,0)=3.18;
    joint4angle=0.00001;
    joint5angle=6.2;


    if(control_type=="PID")
    {
        //控制接收机械臂数据
        socketcan_receive_sub = socketcan_receive.subscribe("received_messages",10,MainpulatorCallback);
        trajectory_receive_sub = socketcan_receive.subscribe("trajectory",10, trajectoryCallback);
    }
    else
    {
        socketcan_receive_sub = socketcan_receive.subscribe("received_messages",10,MainpulatorCallback);
        trajectory_receive_sub = socketcan_receive.subscribe("trajectory",10, trajectoryCallback);
    }

    //发布机械臂当前角度信息
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("chatter",100);  
    
    //以下是角度发布部分，用于向其他控制节点发送机械臂关节信息
    ros::NodeHandle trajectory_pub;
    ros::Publisher Angle_Pub_To_All_Nodes = trajectory_pub.advertise<sensor_msgs::JointState>("Angle_Pub_To_All_Nodes",100);  

    ros::Rate loop_rate(100);
    int socket_can = param::SocketCANInit();
    ROS_INFO("socket_can = %d",socket_can);
    ROS_INFO("control_type  =  %s",control_type.c_str());

     can_msgs::Frame send_message;

    if(control_type=="PID")
    {
        //机械臂输出化配置 
        ROS_INFO("init");
        param::PositionInit(joint1,socket_can);
        param::PositionInit(joint2,socket_can);
        param::PositionInit(joint3,socket_can);
        param::PositionInit(joint4,socket_can);
        param::PositionInit(joint5,socket_can);
        usleep(500000);
        //机械臂使能
        param::Enable(socket_can,6,joint1);
        param::Enable(socket_can,6,joint2);
        param::Enable(socket_can,6,joint3);
        param::Enable(socket_can,6,joint4);
        param::Enable(socket_can,6,joint5);
        usleep(500000);
        ROS_INFO("init end");

    }
    else if(control_type=="ARC"||control_type=="CAMERA"){
        //机械臂力矩模式输出化配置
        //param::MomentInit(joint1,socket_can);
        //param::MomentInit(joint2,socket_can);
        //param::MomentInit(joint3,socket_can);
        //param::PositionInit(joint5,socket_can);
        //机械臂使能
        //param::Enable(socket_can,6,joint1);
        //param::Enable(socket_can,6,joint2);
        //param::Enable(socket_can,6,joint3);
        //param::Enable(socket_can,6,joint5);
        param_init();     
        ROS_INFO("init end");
    }


    q(0,0)=0.00001;
    q(1,0)=0.00001;
    q(2,0)=0.00001;

    int count_control240619=0;
////////////////////////////////////////////
    int running_times=0;
    int running_times_record = 0;
    int running_times_problem = -1;
    double q1_last, q2_last, q1_problem, q2_problem;
    int problem_times = 0;
    bool shuapin = true;


    int switch_thresh = 2200;
    int gripper_current_now = 0.0;
    int gripper_current_last = 0.0;

    while (ros::ok)
    {
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.position.resize(8);
            joint_state.velocity.resize(16);
            joint_state.position[0] =  joint4.get_current_angle();
            joint_state.position[1] = q(0,0) ;
            joint_state.position[2] = q(1,0) ;
            joint_state.position[3] = q(2,0) ;
            joint_state.position[4] = expect_q(0,0);
            joint_state.position[5] = expect_q(1,0);
            joint_state.position[6] = expect_q(2,0);
            joint_state.position[7] = joint5.get_ActualCurrent();

            joint_state.velocity[0] = q(0,0);
            joint_state.velocity[1] = q(1,0);
            joint_state.velocity[2] = q(2,0);
            joint_state.velocity[3] = joint4_actual_angle;
            joint_state.velocity[4] = joint5_actual_angle;
            joint_state.velocity[5] = qd(0,0);
            joint_state.velocity[6] = qd(1,0);
            joint_state.velocity[7] = qd(2,0);
            joint_state.velocity[8] = joint4_actual_velocity;      
            joint_state.velocity[9] = joint5_actual_current;
            joint_state.velocity[10]=Kinematics_x;
            joint_state.velocity[11]=Kinematics_y;
            joint_state.velocity[12]=Kinematics_z;
            joint_state.velocity[13]=PU_status;
            pub.publish(joint_state);
            Angle_Pub_To_All_Nodes.publish(joint_state);
            //ROS_INFO("q1q2q3q4q5 = %f, %f, %f, %f, %f", q(0,0), q(1,0), q(2,0), joint4_actual_angle, joint5_actual_angle);
            //ROS_INFO("240706Q1Q2Q3q4=%f,%f,%f,%f",joint5_actual_angle,joint5_actual_current,Kinematics_z,PU_status);  

        count_time++;
        count1++;
        running_times++;

        count_control240619++;
        if(count1%1000==0){
            ROS_INFO("time = %f", double(count1)*1.0/1000.0);
        }

        count_time_teleope++;
            if(count_time_teleope>=1)
            {
                //Kinematics_expect_q=Kinematics_Solver(expect_q(0,0),expect_q(1,0),expect_q(2,0),KB_W,KB_A,KB_Q,KB_SPEEDUP,1,1,1);
            gripper_current_now =  joint5.get_ActualCurrent();

//导纳

            if(RE_status==1||RE_status==2||RE_status==7)
            {
            //if(gripper_current_last != 0.0 && gripper_current_now - gripper_current_last >= 20 && gripper_current_now >= 0)
            //   gripper_ultimate -= 0.0015;
            if(KB_D>0.01)
            {
                if(gripper_current_now>=switch_thresh)
                    gripper_ultimate -= 0.003;
                else
                    gripper_ultimate += 0.003;
            }
            else if (KB_D<-0.01)
            {
                gripper_ultimate -= 0.003;
            }
            else
            {
                ;
            }

            if(gripper_current_now>=switch_thresh)
                joint5angle = gripper_ultimate - gripper_current_now*0.0000000 - (gripper_current_now-switch_thresh) * 0.00004;
            else if (gripper_current_now<=-switch_thresh)
                joint5angle = gripper_ultimate - gripper_current_now*0.0000000 - (gripper_current_now+switch_thresh) * 0.0000001;
            else
                joint5angle = gripper_ultimate - gripper_current_now*0.0000000;

            
            count_time_teleope = 0;
            }
            }
            gripper_current_last = gripper_current_now;
            



            if(KB_TURNLR>0.01)
            {
                joint4angle=joint4angle+0.0008;
            }
            else if(KB_TURNLR<-0.01)
            {
                joint4angle=joint4angle-0.0008;
            }
            else
            {
                joint4angle=joint4angle;
            }

//计算正运动学
    Kinematics_r = 1000*l1*cos(-q(1,0))+1000*l2*cos(-q(1,0)+q(2,0));
    Kinematics_x=Kinematics_r*cos(q(0,0));
    Kinematics_y=Kinematics_r*sin(q(0,0));
    Kinematics_z=1000*l1*sin(-q(1,0))+1000*l2*sin(-q(1,0)+q(2,0));




        //以下是PID控制部分
        if(control_type=="PID")
        {

    {
    //前进命令，加速阶段A
    double t_KB =0.001;
    if(Velocity_KB<0.25&&KB_W>0)
    {
        q_TeleopeKB=q_TeleopeKB+Velocity_KB*t_KB;
        Aceleration_KB=0.5;
        Velocity_KB=Velocity_KB+Aceleration_KB*t_KB;

    }
        //前进命令，匀速阶段B
    else if(Velocity_KB>=0.25&&KB_W>0)
    {
        q_TeleopeKB=q_TeleopeKB+Velocity_KB*t_KB;
        Aceleration_KB=0.0;
        Velocity_KB=0.25;
    }
    //前进无命令，匀速阶段C
    else if(Velocity_KB>0.01&&KB_W==0)
    {
        q_TeleopeKB=q_TeleopeKB+Velocity_KB*t_KB;
        Aceleration_KB=-0.5;
        Velocity_KB=Velocity_KB+Aceleration_KB*t_KB;
    }
    //后退命令，加速阶段
    else if(Velocity_KB>-0.25&&KB_W<0)
    {
        q_TeleopeKB=q_TeleopeKB+Velocity_KB*t_KB;
        Aceleration_KB=-0.5;
        Velocity_KB=Velocity_KB+Aceleration_KB*t_KB;
    }
    //后退命令，匀速阶段
    else if(Velocity_KB<=-0.25&&KB_W<0)
    {
        q_TeleopeKB=q_TeleopeKB+Velocity_KB*t_KB;
        Aceleration_KB=0.0;
        Velocity_KB=-0.25;

    }
   //前进无命令，匀速阶段
    else if(Velocity_KB<-0.01&&KB_W==0)
    {
        q_TeleopeKB=q_TeleopeKB+Velocity_KB*t_KB;
        Aceleration_KB=0.5;
        Velocity_KB=Velocity_KB+Aceleration_KB*t_KB;
    }
    else if(Velocity_KB<=0.01&&Velocity_KB>=-0.01&&KB_W==0)
    {
        Aceleration_KB=0.0;
        Velocity_KB=0.0;
        q_TeleopeKB=q_TeleopeKB;
    }
    else{
        Aceleration_KB=0.0;
        Velocity_KB=0.0;
        q_TeleopeKB=q_TeleopeKB;
    }

    //ROS_INFO("QVA  = %lf , %lf , %lf", q_TeleopeKB, Velocity_KB, Aceleration_KB);

    }
    
                can_msgs::Frame frames;
                //键盘控制关节空间
                //send_message =joint1.set_angle(expect_q(0,0)+q_TeleopeKB);
                //send_message = joint3.set_angle(expect_q(2,0)+q_TeleopeKB);
                //send_message = joint2.set_angle(expect_q(0,0)+q_TeleopeKB);

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
                send_message = joint3.set_angle(expect_q(2,0));
                socketcan_send_pub.publish(send_message);

                if(count_control240619>=1)
                {
                count_control240619=0;
                if(joint4angle<-3.1)
                {
                    joint4angle=-3.1;
                }
                if(joint4angle>3.1)
                {
                    joint4angle=3.1;
                }
                send_message = joint4.set_angle_for_new_joint(joint4angle);
                socketcan_send_pub.publish(send_message);
                send_message =joint4.set_angle(joint4angle);
                socketcan_send_pub.publish(send_message);

                if(joint5angle<0.00001)
                {
                    joint5angle=0.00001;
                }
                if(joint5angle>6.2)
                {
                    joint5angle=6.2;
                }
                send_message = joint5.set_angle_for_new_joint(joint5angle);
                socketcan_send_pub.publish(send_message);
                send_message =joint5.set_angle(joint5angle);
                socketcan_send_pub.publish(send_message);
                }
                frames.id = 0x80;
	            frames.dlc = 0;
                socketcan_send_pub.publish(frames);         

        }



            ros::spinOnce();


        if(loop_rate.sleep())
        {
            if(shuapin)
            {
                //cout<<"#"<<running_times<< endl;
            }
            //running_times_record = running_times;
        }
        else
        {
            if(shuapin)
            {
                cout<<"####################"<<endl;
            }
        }
    }
////////////////////////////////////////////////////get_ActualCurrent

}


//机械臂内部返回实绩角度的回调函数，用于取得实际q 实际dq
void MainpulatorCallback(const can_msgs::Frame& receive_message)
{
    if(control_type=="PID")
    {
    switch (receive_message.id)
    {
        case 0x181:
            joint1.current_angle(receive_message);
            q(0,0) = joint1.get_current_angle();
            break;
        case 0x182:
            joint2.current_angle(receive_message);
            q(1,0) = joint2.get_current_angle();
            break;
        case 0x183:
            joint3.current_angle(receive_message);
            q(2,0) = joint3.get_current_angle();
            break;
        case 0x184:
            joint4.current_angle(receive_message);
            joint4_actual_angle= joint4.get_current_angle();
            break;
        case 0x185: 
            joint5.current_angle(receive_message);
            joint5_actual_angle= joint5.get_current_angle();
            break;
            
        case 0x281: 
            joint1.current_velocity(receive_message);
            qd(0,0) = joint1.get_current_velocity();
            break;
        case 0x282: 
            joint2.current_velocity(receive_message);
            qd(1,0) = joint2.get_current_velocity();
            break;
        case 0x283: 
            joint3.current_velocity(receive_message);
            qd(2,0) = joint3.get_current_velocity();
            break;
        case 0x284: 
            joint4.current_velocity(receive_message);
            joint4_actual_velocity= joint4.get_current_velocity();
            break;
        case 0x285: 
            joint5.ActualCurrent(receive_message);
            joint5_actual_current = joint5.get_ActualCurrent();
            break;

        default:
            break;
    }
    }
    else{
    switch (receive_message.id)
    {
        case 0x181:
            joint1.current_angle(receive_message);
            q(0,0) = joint1.get_current_angle();
            break;
        case 0x182:
            joint2.current_angle(receive_message);
            q(1,0) = joint2.get_current_angle();
            break;
        case 0x183:
            joint3.current_angle(receive_message);
            q(2,0) = joint3.get_current_angle();
            break;
        case 0x184:
            joint4.current_angle(receive_message);
            joint4_actual_angle= joint4.get_current_angle();
            break;
        case 0x185:
            joint5.current_angle(receive_message);
            joint5_actual_angle = joint5.get_current_angle();
            break;


        case 0x281:
            joint1.current_velocity(receive_message);
            qd(0,0) = joint1.get_current_velocity();
            break;
        case 0x282:
            joint2.current_velocity(receive_message);
            qd(1,0) = joint2.get_current_velocity();
            break;  
        case 0x283:
            joint3.current_velocity(receive_message);
            qd(2,0) = joint3.get_current_velocity();
        case 0x284:
            joint4.current_velocity(receive_message);
            joint4_actual_velocity = joint4.get_current_velocity();
        case 0x285: 
            joint5.ActualCurrent(receive_message);
            joint5_actual_current = joint5.get_ActualCurrent();
            break;
        default:
            break;
    }
    }
}

//pid,arc回调
void trajectoryCallback(const sensor_msgs::JointState& joint_state){
    expect_q1 = joint_state.position[0];
    expect_dq1 = joint_state.velocity[0];
    expect_ddq1 = joint_state.effort[0];

    expect_q2 = joint_state.position[0];
    expect_dq2 = joint_state.velocity[0];
    expect_ddq2 = joint_state.effort[0];

    
    expect_q3 = joint_state.position[0];
    expect_dq3 = joint_state.velocity[0];
    expect_ddq3 = joint_state.effort[0];
    enpect_q5 = joint_state.position[1];

    expect_q(0,0) =  joint_state.position[0];
    expect_q(1,0) =  joint_state.position[0];
    expect_q(2,0) =  joint_state.position[0];

    expect_qd(0,0) = joint_state.velocity[0];
    expect_qd(1,0) = joint_state.velocity[0];
    expect_qd(2,0) = joint_state.velocity[0];

    expect_qdd(0,0) = joint_state.effort[0];
    expect_qdd(1,0) = joint_state.effort[0];
    expect_qdd(2,0) = joint_state.effort[0];

    if( joint_state.effort[1]-2<=0.01)
        trajectory_flag = 1;
       // ROS_INFO("expect_q1 = %lf",expect_q1);
}


void TeleopeKB_Callback(const geometry_msgs::Twist & msg){
    KB_W = msg.linear.x;
    KB_A = msg.linear.y; 
    KB_Q = msg.linear.z; 
}

void TeleopeJOY_Callback(const geometry_msgs::Twist & msg){
    KB_W = msg.linear.x;
    KB_A = msg.linear.y; 
    KB_Q = msg.linear.z; 
    KB_D=msg.angular.x;
    KB_SPEEDUP=msg.angular.y;
    KB_TURNLR=msg.angular.z;
}

void Imu_Info_Callback(const sensor_msgs::JointState & msg){
    imu_R = msg.position[0];
    imu_P = msg.position[1];
    imu_Y = msg.position[2];
    expect_q(0,0) = msg.effort[9];
    expect_q(1,0) = msg.effort[10];
    expect_q(2,0) = -msg.effort[11];
}


//pid,arc回调,9参数
//文文遥操作相关内容，，
void TeleOperationCallback(const  sensor_msgs::Imu& msg){
    expect_q(0,0) = msg.orientation.x;
    expect_q(1,0) = msg.orientation.y;
    expect_q(2,0) = msg.orientation.z;
    joint5_catch_TeleOpe=msg.orientation.w;
    //ROS_INFO("catch=  %f ",msg.orientation.w);
    //ROS_INFO("Tele_Ope_q1=  %f ",msg.orientation.x);
    //ROS_INFO("Tele_Ope_q2=  %f ",msg.orientation.y);
    //ROS_INFO("Tele_Ope_q3=  %f ",msg.orientation.z);

    expect_qd(0,0) = msg.angular_velocity.x;
    expect_qd(1,0) = msg.angular_velocity.y;
    expect_qd(2,0) = msg.angular_velocity.z;

    expect_qdd(0,0) = msg.linear_acceleration.x;
    expect_qdd(1,0) = msg.linear_acceleration.y;
    expect_qdd(2,0) = msg.linear_acceleration.z;
}

//申昊显控信息回调
void QGC_Callback(const sensor_msgs::JointState & msg){
    RE_q1=msg.velocity[0];
    RE_q2=msg.velocity[1];
    RE_q3=msg.velocity[2];
    RE_q4=msg.velocity[3];
    RE_q5=msg.velocity[4];
    RE_qd1=msg.velocity[5];
    RE_qd2=msg.velocity[6];
    RE_qd3=msg.velocity[7];
    RE_qd4=msg.velocity[8];
    RE_Current5=msg.velocity[9];
    RE_KB_W=msg.velocity[10];
    RE_KB_A=msg.velocity[11];
    RE_KB_S=msg.velocity[12];
    RE_KB_D=msg.velocity[13];
    RE_KB_Q=msg.velocity[14];
    RE_KB_E=msg.velocity[15];
    RE_KB_O=msg.velocity[16];
    RE_KB_C=msg.velocity[17];
    RE_KB_R=msg.velocity[18];
    RE_KB_L=msg.velocity[19];
    RE_status==msg.velocity[20]? (RE_status_changed = false) : (RE_status_changed = true);
    RE_status=msg.velocity[20];
    RE_Buttom=msg.velocity[21];

    //急停
    if (RE_status==0)
    {    
        RE_status_lock_st1=0;
        RE_status_lock_st2=0;
        RE_status_lock_st7=0;
       //急停     
        PU_status=0;
        if(RE_status_lock_st0==0)
        {
            INTTT_q1=(int)(q(0,0)*10000);
            INTTT_q2=(int)(q(1,0)*10000);
            INTTT_q3=(int)(q(2,0)*10000);
            INTTT_q4=(int)(joint4_actual_angle*10000);
            INTTT_q5=(int)(joint5_actual_angle*10000);
            RE_status_lock_st0=1;
        }

        expect_q(0,0)=((double)INTTT_q1)/10000;
        expect_q(1,0)=((double)INTTT_q2)/10000;
        expect_q(2,0)=((double)INTTT_q3)/10000;
        joint4angle=((double)INTTT_q4)/10000;
        joint5angle=((double)INTTT_q5)/10000;

        gripper_ultimate=joint5angle;
        //ROS_INFO("stop240706"); 
        //ROS_INFO("joint3angle=%f",expect_q(2,0));  
    }
    //遥操作
    else if(RE_status==1){
        //遥操作
        RE_status_lock_st0=0;
        RE_status_lock_st2=0;
        RE_status_lock_st7=0;
        PU_status=1;
        if(RE_status_lock_st1==0)
        {
            expect_q(0,0)=q(0,0);
            expect_q(1,0)=q(1,0);
            expect_q(2,0)=q(2,0);
            //INTTT_q4=(int)(joint4_actual_angle*10000);
            //INTTT_q5=(int)(joint5_actual_angle*10000);
            RE_status_lock_st1=1;
        }
        //左右 左-1 右1
        if(RE_qd1>=5 &&abs(RE_qd1)>abs(RE_qd2) && abs(RE_qd1)>abs(RE_qd3)){
            KB_W=-1;
        }
        else if(RE_qd1<=-5 &&abs(RE_qd1)>abs(RE_qd2) && abs(RE_qd1)>abs(RE_qd3)){
            KB_W=1; 
        }
        else{
            KB_W=0; 
        }
        //前后 上-1 下1
        if(RE_qd2>=5 &&abs(RE_qd2)>abs(RE_qd1) && abs(RE_qd2)>abs(RE_qd3)){
            KB_A=1;
        }
        else if(RE_qd2<=-5&&abs(RE_qd2)>abs(RE_qd1) && abs(RE_qd2)>abs(RE_qd3)){
            KB_A=-1; 
        }
        else{
            KB_A=0; 
        }
        //上下 前-1 后1
        if(RE_qd3>=5&&abs(RE_qd3)>abs(RE_qd1) && abs(RE_qd3)>abs(RE_qd2)){
            KB_Q=-1;
        }
        else if(RE_qd3<=-5 &&abs(RE_qd3)>abs(RE_qd1) && abs(RE_qd3)>abs(RE_qd2)){
            KB_Q=1; 
        }
        else{
            KB_Q=0; 
        }

        if(RE_q4<-2.4){
            KB_TURNLR=-1;
        }
        else if(RE_q4>2.4){
            KB_TURNLR=1; 
        }
        else{
            KB_TURNLR=0; 
        }

        //抓手
        if(RE_Buttom==1){
            KB_D=1;
        }
        else if(RE_Buttom==2){
            KB_D=-1; 
        }
        else{
            KB_D=0; 
        }

//    左右 前后 上下    /左右  前后  上下
        Kinematics_expect_q=Kinematics_Solver(expect_q(0,0),expect_q(1,0),expect_q(2,0),KB_W,KB_A,KB_Q,1.0,RE_qd1*0.025,RE_qd2*0.07,RE_qd3*0.04);
        expect_q=Kinematics_expect_q;
        //ROS_INFO("Kinematics_44444444444=%f,%f,%f,%f",RE_q4,KB_TURNLR,RE_Buttom,KB_D);  
    }
    //键盘笛卡尔空间操作
    else if(RE_status==2){
        RE_status_lock_st0=0;
        RE_status_lock_st1=0;
        RE_status_lock_st7=0;
        PU_status=2;

        if(RE_status_lock_st2==0)
        {
            expect_q(0,0)=q(0,0);
            expect_q(1,0)=q(1,0);
            expect_q(2,0)=q(2,0);
            //INTTT_q4=(int)(joint4_actual_angle*10000);
            //INTTT_q5=(int)(joint5_actual_angle*10000);
            RE_status_lock_st2=1;
        }

        //前后
        if(RE_KB_W==1&&RE_KB_S==0){
            KB_Q=-1;
        }
        else if(RE_KB_W==0&&RE_KB_S==1){
            KB_Q=1; 
        }
        else{
            KB_Q=0; 
        }
        //左右
        if(RE_KB_A==1&&RE_KB_D==0){
            KB_W=-1;
        }
        else if(RE_KB_A==0&&RE_KB_D==1){
            KB_W=1; 
        }
        else{
            KB_W=0; 
        }
        //上下
        if(RE_KB_Q==1&&RE_KB_E==0){
            KB_A=-1;
        }
        else if(RE_KB_Q==0&&RE_KB_E==1){
            KB_A=1; 
        }
        else{
            KB_A=0; 
        }
        //转动
        if(RE_KB_L==1&&RE_KB_R==0){
            KB_TURNLR=1;
        }
        else if(RE_KB_L==0&&RE_KB_R==1){
            KB_TURNLR=-1; 
        }
        else{
            KB_TURNLR=0; 
        }
        //抓手
        if(RE_KB_O==1&&RE_KB_C==0){
            KB_D=1;
        }
        else if(RE_KB_O==0&&RE_KB_C==1){
            KB_D=-1; 
        }
        else{
            KB_D=0; 
        }
        //ROS_INFO("INTTT_qqqqqq=%f,%f,%f",expect_q(0,0),expect_q(1,0),expect_q(2,0));  
        Kinematics_expect_q=Kinematics_Solver(expect_q(0,0),expect_q(1,0),expect_q(2,0),KB_W,KB_A,KB_Q,1.0,1.0,1.0,1.0);
        expect_q=Kinematics_expect_q;
        //ROS_INFO("240710info=%f,%f,%f,%f,%f",joint5angle,joint5_actual_angle,RE_status,KB_TURNLR,KB_D);  
        //ROS_INFO("Kinematics_33333333=%f,%f,%f",RE_KB_O,RE_KB_C,KB_D);  
        //ROS_INFO("Kinematics_expect_q=%f,%f,%f,%f",expect_q(0,0),expect_q(1,0),expect_q(2,0),PU_status);  
    }
    //收回？
    else if(RE_status==3){
        gripper_ultimate=joint5_actual_angle;
        RE_status_lock_st0=0;
        RE_status_lock_st1=0;
        RE_status_lock_st2=0;
        RE_status_lock_st7=0;
        //最终在0，0，3.14，0，6.2的收回位置
        PU_status=3;
        if(RE_status_changed){
            ROS_INFO("status==changed ");
            Status3_lock=0;
        }

        joint4angle=0.0001;
        joint5angle=6.2;

        if((q(1,0)-q(2,0)>=0&&(Status3_lock==0||Status3_lock==1))||Status3_lock==1 ){
            Status3_lock=1;
            ROS_INFO("DDDDDDDDD===  ,%f ,%f,%f,%f ",q(0,0),q(1,0),q(2,0),Status3_lock);
            if(abs(joint5_actual_angle-6.2)<0.05)
            {
                expect_q(1,0)=1.918889;
            }
            else{
                //expect_q(2,0)=((int)(q(2,0)*100))/100;
            }

            if(abs(q(1,0)-1.918889)<0.05&&abs(joint5_actual_angle-6.2)<0.05)
            {
                expect_q(2,0)=3.14;
            }
            else{
                //expect_q(1,0)=((int)(q(2,0)*100))/100;
            }

            if(abs(q(2,0)-3.14)<0.05&&abs(joint5_actual_angle-6.2)<0.05)
            {
                expect_q(1,0)=1.57;
            }
            else{
                //expect_q(0,0)=((int)(q(0,0)*100))/100;
            }

            if(abs(q(1,0)-1.57)<0.05&&abs(q(2,0)-3.14)<0.05&&abs(joint5_actual_angle-6.2)<0.05)
            {
                expect_q(0,0)=0.0001;
            }
            else{
                ;
            }

            if(abs(q(0,0)-0.0001)<0.05&&abs(q(2,0)-3.14)<0.05&&abs(joint5_actual_angle-6.2)<0.05)
            {
                expect_q(1,0)=0.0001;
            }
            else{
                ;
            }
        }
        //E
        else if((q(1,0)-q(2,0)<0&&(Status3_lock==0||Status3_lock==2))||Status3_lock==2){
            Status3_lock=2;
            ROS_INFO("EEEEEEEEEEE===  ,%f ,%f,%f,%f ",q(0,0),q(1,0),q(2,0),Status3_lock);
            if(abs(joint5_actual_angle-6.2)<0.05)
            {
                expect_q(2,0)=3.14;
            }
            else{
                //expect_q(2,0)=((int)(q(2,0)*100))/100;
            }
            if(abs(q(2,0)-3.14)<0.05&&abs(joint5_actual_angle-6.2)<0.05)
            {
                expect_q(1,0)=1.57;
            }
            else{
                //expect_q(1,0)=((int)(q(2,0)*100))/100;
            }

            if(abs(q(1,0)-1.57)<0.05&&  abs(q(2,0)-3.14)<0.05 &&abs(joint5_actual_angle-6.2)<0.05)
            {
                expect_q(0,0)=0.0001;
            }
            else{
                //expect_q(0,0)=((int)(q(0,0)*100))/100;
            }
            }

            if(abs(q(0,0)-0.0001)<0.05&&abs(q(2,0)-3.14)<0.05&&abs(joint5_actual_angle-6.2)<0.05)
            {
                expect_q(1,0)=0.0001;
            }
            else{
                ;
            }

            if(abs(q(0,0)-0.0001)<0.05 &&abs(q(1,0)-0.0001)<0.05 &&abs(q(2,0)-3.14)<0.05 && abs(joint5_actual_angle-6.2)<0.05)
            {
            PU_status=5;
            Status3_lock=0;
            }
            else{
                ;
            }
        }
    //伸出？
    else if(RE_status==4){
        gripper_ultimate=joint5_actual_angle;
        RE_status_lock_st0=0;
        RE_status_lock_st1=0;
        RE_status_lock_st2=0;
        RE_status_lock_st7=0;
        if(RE_status_changed){
            ROS_INFO("status==changed ");
            Status4_lock=0;
        }
        //最终在1.57，1.57，1.57，0，0的生出位置     
        PU_status=4;
        //从开机/收回状态开始伸出
        if((abs(q(0,0)-0.0001)<0.05 &&abs(q(1,0)-0.0001)<0.05 && abs(q(2,0)-3.14)<0.08 && abs(joint5_actual_angle-6.2)<0.05 &&(Status4_lock==0||Status4_lock==1))||Status4_lock==1){
            Status4_lock=1;
            ROS_INFO("AAAAAA===  ,%f ,%f,%f,%f ",q(0,0),q(1,0),q(2,0),Status4_lock);
            expect_q(1,0)=1.57;
            joint4angle=0.0001;

            if(abs(q(1,0)-1.57)<0.05)
            {
                expect_q(0,0)=1.57;
            }
            else{
                ;
            }
            if(abs(q(0,0)-1.57)<0.05)
            {
                expect_q(1,0)=1.918889;
            }
            else{
                ;
            }
            if(abs(q(1,0)-1.918889)<0.05 && abs(q(0,0)-1.57)<0.05)
            {
                expect_q(2,0)=1.918889;
            }
            else{
                ;
            }
            if(abs(q(0,0)-1.57)<0.05 &&abs(q(1,0)-1.918889)<0.05 &&abs(q(2,0)-1.918889)<0.05)
            {
                joint5angle=0.0001;
            }
            else{
                ;
            }
        }
        else{
            //从三关节比较靠外的状态开始伸出
            if((q(1,0)-q(2,0)<=0&&(Status4_lock==0||Status4_lock==3))||Status4_lock==3)
            {
                Status4_lock=3;
                ROS_INFO("CC===  ,%f ,%f,%f,%f ",q(0,0),q(1,0),q(2,0),Status4_lock);
                joint4angle=0.0001;
                if(abs(joint4_actual_angle-0.0001)<0.05){
                    expect_q(2,0)=2.826;
                }
                if(abs(q(2,0)-2.826)<0.05 && abs(joint4_actual_angle-0.0001)<0.05)
                {
                    expect_q(1,0)=1.91889;
                }
                else{
                    ;
                }
                if( abs(q(1,0)-1.91889)<0.05&&abs(q(2,0)-2.826)<0.05 && abs(joint4_actual_angle-0.0001)<0.05)
                {
                    expect_q(0,0)=1.57;
                }
                else{
                    ;
                }
                if(abs(q(1,0)-1.91889)<0.05&&abs(q(0,0)-1.57)<0.05 && abs(joint4_actual_angle-0.0001)<0.05)
                {
                    expect_q(2,0)=1.918889;
                }
                else{
                    ;
                }
                if(abs(q(0,0)-1.57)<0.05 &&abs(q(1,0)-1.918889)<0.05 &&abs(q(2,0)-1.918889)<0.05)
                {
                    joint5angle=0.0001;
                }
                else{
                    ;
                }
            }
            //从三关节比较靠内的状态（如抓小车）开始伸出
            else if((q(1,0)-q(2,0)>0&&(Status4_lock==0||Status4_lock==2))||Status4_lock==2)
            {
                Status4_lock=2;
                ROS_INFO("BBBB===  ,%f ,%f,%f,%f ",q(0,0),q(1,0),q(2,0),Status4_lock);
                expect_q(2,0)=0.0001;
                joint4angle=0.0001;
                if(abs(q(2,0)-0.0001)<0.05)
                {
                    expect_q(1,0)=1.918889;
                }
                else{
                    ;
                }
                if(abs(q(1,0)-1.91889)<0.05)
                {
                    expect_q(0,0)=1.57;
                }
                else{
                    ;
                }
                if(abs(q(1,0)-1.918889)<0.05 && abs(q(0,0)-1.57)<0.05)
                {
                    expect_q(2,0)=1.918889;
                }
                else{
                    ;
                }
                if(abs(q(0,0)-1.57)<0.05 &&abs(q(1,0)-1.918889)<0.05 &&abs(q(2,0)-1.918889)<0.05)
                {
                    joint5angle=0.0001;
                }
                else{
                    ;
                }
            }
        }
        if(abs(q(0,0)-1.57)<0.05 &&abs(q(1,0)-1.918889)<0.05 &&abs(q(2,0)-1.918889)<0.05 && abs(joint5_actual_angle-0.0001)<0.05)
        {
            PU_status=6;
            Status4_lock=0;
        }
        else{
            ;
        }
    }
    //键盘关节空间操作
    else if(RE_status==7){
        RE_status_lock_st0=0;
        RE_status_lock_st1=0;
        RE_status_lock_st2=0;
        PU_status=7;
        if(RE_status_lock_st7==0)
        {
            expect_q(0,0)=q(0,0);
            expect_q(1,0)=q(1,0);
            expect_q(2,0)=q(2,0);
            //INTTT_q4=(int)(joint4_actual_angle*10000);
            //INTTT_q5=(int)(joint5_actual_angle*10000);
            RE_status_lock_st7=1;
        }

        if(RE_KB_A==1&&RE_KB_D==0)
        {
            expect_q(0,0)=expect_q(0,0)+0.0008;
        }
        else if(RE_KB_A==0&&RE_KB_D==1)
        {
            expect_q(0,0)=expect_q(0,0)-0.0008;
        }
        else
        {
            expect_q(0,0)=expect_q(0,0);
        }

        if(expect_q(0,0)<-PI/7 ){
            expect_q(0,0)=-PI/7;
        }
            else if(expect_q(0,0)>PI*9/12 ){
            expect_q(0,0)=PI*9/12;
        }

        

        if(RE_KB_W==0&&RE_KB_S==1)
        {
            expect_q(1,0)=expect_q(1,0)+0.0005;
        }
        else if(RE_KB_W==1&&RE_KB_S==0)
        {
            expect_q(1,0)=expect_q(1,0)-0.0005;
        }
        else
        {
            expect_q(1,0)=expect_q(1,0);
        }


        if(RE_KB_Q==1&&RE_KB_E==0)
        {
            expect_q(2,0)=expect_q(2,0)+0.0005;
        }
        else if(RE_KB_Q==0&&RE_KB_E==1)
        {
            expect_q(2,0)=expect_q(2,0)-0.0005;
        }
        else
        {
            expect_q(2,0)=expect_q(2,0);
        }

        //转动
        if(RE_KB_L==1&&RE_KB_R==0){
            KB_TURNLR=1;
        }
        else if(RE_KB_L==0&&RE_KB_R==1){
            KB_TURNLR=-1; 
        }
        else{
            KB_TURNLR=0; 
        }
        //抓手
        if(RE_KB_O==1&&RE_KB_C==0){
            KB_D=1;
        }
        else if(RE_KB_O==0&&RE_KB_C==1){
            KB_D=-1; 
        }
        else{
            KB_D=0; 
        }

    }
    //状态锁重置🔓
    else{
        RE_status_lock_st0=0;
        RE_status_lock_st1=0;
        RE_status_lock_st2=0;
        RE_status_lock_st7=0;
        PU_status=888;
    }
}

//非arc，一个类滑膜控制
double arc_control(double expect_q,double expect_dq,double expect_ddq,double q,double dq,double t ,double m,double c,double g){


        double k2 = 5;
        double k1 =  5;



        double err = (q - expect_q);
        double derr = (dq - expect_dq);
        double s = derr + k2 * err;

        //dthp1 = 0;
        dthp1 = -gamma1*((expect_ddq-k1*derr)*s)*t;
        dthp2 = -gamma2*(dq*s)*t;
        dthp3 = -gamma3*atan2(900*dq,1)*2/(M_PI)*s*t;
        dthp4 = gamma4*s*t;

        double th_min1 = 0.1;
	    double th_max1 = 20;
	    double th_min2 = 0;
	    double th_max2 = 50;
	    double th_min3 = 0;
	    double th_max3 =50;
        double th_min4 = -10;
	    double th_max4 =10;

        if(th_max1<=thp1&&dthp1>0)
		    dthp1 = 0;
	    if(thp1<=th_min1&&dthp1<0)
		    dthp1 = 0;

	    if(th_max2<=thp2&&dthp2>0)
		    dthp2 = 0;
	    if(thp2<=th_min2&&dthp2<0)
		    dthp2 = 0;
	
	    if(th_max3<=thp3&&dthp3>0)
		    dthp3 = 0;
	    if(thp3<=th_min3&&dthp3<0)
		    dthp3 = 0;
	    if(th_max4<=thp4&&dthp4>0)
		    dthp4 = 0;
	    if(thp4<=th_min4&&dthp4<0)
		    dthp4 = 0;

	    thp1+= dthp1;
	    thp2 += dthp2;
	    thp3 += dthp3;
        thp4 += dthp4;
        ua_1 = -((-1)*thp1*(expect_ddq) -  thp2 * expect_dq -atan2(900*expect_dq,1)*2/(M_PI)*thp3+ thp4);

       // ua = -((-1)*thp1*(expect_ddq) +  thp2 * expect_dq -atan2(900*expect_dq,1)*2/(M_PI)*thp3+ thp4);
        us3 = 10*expect_dq;
        if(expect_dq>0)
        {
            us3+=10;
        }
        else if(expect_dq<0){
            us3-=10;
        }
        us1_1 = - k1*th_max1 * s;
        u_1 = us1_1 + ua_1;// + 10/(1+exp(-s));
     /*   if(u>0&&u<=7)
        {
            u+=7;
        }
        else if(u<0&&u>=-7)
        {
            u-=7;
        }*/
        return u_1;

    }
    
void  param_init()
{
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            k1(i,j) = 0;
            k2(i,j) = 0;
        }
    }
    //参数表
    k1(0,0) = 5.2;//6.5;
    k1(1,1)  = 7.5;//5.2;//8.0;
    k1(2,2) = 4.6;//4.6
    k2(0,0)  = 5.0;//30.0;
    k2(1,1)  = 5.0;//40.0;
    k2(2,2) = 4.0;//7.0      //15.0;


  
    for(int i=0;i<11;i++)
    {
        theta_d(i,0) = 0;
    }



   theta(0,0) = 13.5;//24;//8;
    theta(1,0) =2.5;//6.5;// 5;

    //B
    theta(2,0) =0;//-7;
    theta(3,0) =0;//-2;// -7;
    
    theta(4,0)  =0;// -7;
    //Df1
    theta(5,0) =0;//9.2;
    theta(6,0) =0;// 15;//15;//12;
    theta(7,0) = 0;//12.5;
    //d
    theta(8,0) = 0;//(-15~15)

    theta(9,0) = 0;//(-80~-30)

    theta(10,0) = 0;//(-5~20)

/*
    slidetheta(0,0) = m1;
    slidetheta(1,0) = m2;

    //B
    slidetheta(2,0) =0;
    slidetheta(3,0) =0;// -7;
    
    slidetheta(4,0)  = 0;
    //Df1
    slidetheta(5,0) =0;
    slidetheta(6,0) = 0;//15;//12;
    slidetheta(7,0) = 0;
    //d
    slidetheta(8,0) = 0;

    slidetheta(9,0) = 0;

    slidetheta(10,0) = 0;

*/

    theta_min(0,0) = 1;
    theta_max(0,0) = 16;
    theta_min(1,0) = 0.1;
    theta_max(1,0) = 10;
//B1
    theta_min(2,0) = -100;//-10;
    theta_max(2,0) = 100;//10;

//B2
    theta_max(3,0) = 50;//10;
    theta_min(3,0)  = -50;//-2;
    //B3
    theta_max(4,0) = 30;//3;
    theta_min(4,0)  =-300;//-10;
    //Df1
    theta_max(5,0) = 100;//10;
    theta_min(5,0)  = -20;//-2;
    //Df2
    theta_max(6,0) = 50;//20;
    theta_min(6,0)  = -10;//13.5;
    //Df3
    theta_max(7,0) =20;// 15;
    theta_min(6,0)  = 0;//0;
    //d1
    theta_max(8,0) = 100;//50;
    theta_min(8,0)  =-100;// -50;
    //d2
    theta_max(9,0) = 50;//50;
    theta_min(9,0)  =-50;// -50;
        //d2
    theta_max(10,0) = 50;//50;
    theta_min(10,0)  =-50;// -50;

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            theta_max1(i,j) =0;
            Xite(i,j) = 0;
        }
    }
    theta_max1(0,0) = 20;//20;
    theta_max1(1,1) = 20;//20;
    theta_max1(2,2) = 20;//20;
    Xite(0,0) = 10;
    Xite(1,1) = 10;
    Xite(2,2) = 10;

    
}

Vector3d arc(Vector3d& expect_q, Vector3d& expect_qd, Vector3d& expect_qdd,Vector3d& q,Vector3d& qd,double t)
{
    gammamamm(0,0) = 50;//10;//650;//650;
    gammamamm(1,1) = 20;//10;//250;//250;

    gammamamm(2,2) = 0;//1000;//1000;
    gammamamm(3,3) = 0;//1000;//1000;
    gammamamm(4,4) = 0;//1000;//1000;

    gammamamm(5,5) = 0;//1000;//1000;
    gammamamm(6,6) = 0;//1000;//1000;
    gammamamm(7,7) = 0;//1000;// 1000;
    
    gammamamm(8,8) = 5000;//6000;//6000
    gammamamm(9,9) = 2000;//2000;//2000
    gammamamm(10,10) = 2000;//2000;//3000


    //Matrix3d  err(2.1);
    err = q-expect_q;
    derr = qd-expect_qd;
    Vector3d s = derr + k2*err;
    qr = expect_qd -k2*err ;
    dqr = expect_qdd - k2*derr;
    //ROS_INFO(" dqr(0,0) = %lf,k2(0,0)*derr = %lf,qd(0,0)  = %lf",derr(0,0),k2(0,0)*derr(0,0),qd(0,0) );
    qr_dot = expect_qd ;//- k2*0;
    dqr_dot = expect_qdd;// - k2*0;

  //m1 m2

    fai(0,0) = -1.0*(1.0 / 3.0 * l1 * l1 *cos(q(1,0))*cos(q(1,0)))* dqr(0,0);

    fai(0,1) =  -1.0* ((1.0/ 12.0 * l2 * l2*cos(q(1,0)-q(2,0)) *cos(q(1,0)-q(2,0))  + l1 * l1 * cos(q(1,0)) * cos(q(1,0)) + 1.0/ 4.0 * l2 * l2 * cos(q(1,0) -q(2,0)) * cos(q(1,0) - q(2,0)) + l1 * l2 * cos(q(1,0)) * cos(q(1,0) - q(2,0))) * dqr(0,0) +
        (-2.0 * l1 * l1 * cos(q(1,0)) * sin(q(1,0)) - 2.0 / 3.0 * l2 * l2 * cos(q(1,0) - q(2,0)) * sin(q(1,0) - q(2,0)) - l1 * l2*sin(2*q(1,0) - q(2,0)) ) * qd(1,0) * qr(0,0) +
        (2.0 / 3.0 * l2 * l2 * sin(q(1,0) - q(2,0)) * cos(q(1,0) - q(2,0)) + l1 * l2 * cos(q(1,0)) * sin(q(1,0) - q(2,0))) * qd(2,0) * qr(0,0));
    fai(0,2) = -qd(0,0);
    fai(0,3) = 0;

    fai(0,4) = 0;
    fai(0,5) = -atan2(900*qd(0,0),1)*2.0/(M_PI*1.0);
    fai(0,6) = 0;
    fai(0,7) = 0;
    fai(0,8) = 1;    
    fai(0,9) = 0;
    fai(0,10) = 0;

    fai(1,0) = -1.0 * ((1.0 / 3.0 * l1 * l1 * dqr(1,0)) + 1.0 / 2.0 * g * l1 * cos(q(1,0))+1.0/3.0*l1*l1*cos(q(1,0))*sin(q(1,0))*qd(0,0) * qr(0,0));
// df1 df2
    fai(1,1) = -1.0 * ((1.0 / 3.0 * l2 * l2 + l1 * l1 + l1 * l2 * cos(q(2,0))) * dqr(1,0) -  l1 * l2 * sin(q(2,0)) * qd(2,0) * qr(1,0) + (-1.0 / 3.0 * l2 * l2 - 0.5 * l1 * l2 * cos(q(2,0))) * dqr(2,0) +
        0.5 * l1 * l2 * sin(q(2,0)) * qd(2,0) * qr(2,0) +(l1 * l1 * cos(q(1,0)) * sin(q(1,0)) + 0.5*l1 * l2 * sin(2 * q(1,0) - q(2,0)) + 1.0/3.0* l2 * l2 * cos(q(1,0) - q(2,0)) * sin(q(1,0) - q(2,0))) * qd(0,0) * qr(0,0) +
        g * l1 * cos(q(1,0))  + 0.5 * g * l2 * cos(q(1,0) - q(2,0)));

    fai(1,2) = 0;
    fai(1,3) = -qd(1,0);

    fai(1,4) = 0;
    fai(1,5) = 0;
    fai(1,6) = -atan2(900*qd(1,0),1)*2.0/(M_PI*1.0);
    fai(1,7) = 0;
    fai(1,8) = 0;    
    fai(1,9) = 1;
    fai(1,10) = 0;

    fai(2,0) = 0;
    fai(2,1) = -1 * (1.0 / 3.0 * l2 * l2 * dqr(2,0) + (-1.0 / 3.0 * l2 * l2 - 0.5 * l1 * l2 * cos(q((2,0)))) * dqr(1,0) +
       ( 1.0/3.0* l2 * l2 * cos(q(1,0) - q(2,0)) * sin(q(1,0) - q(2,0)) + 0.5 * l1 * l2 * cos(q(1,0)) * sin(q(1,0) - q(2,0)) )* qd(0,0) * qr(0,0) +
        0.5 * l1 * l2 * sin(q(2,0)) * qd(1,0) * qr(1,0) - 0.5 * g * l2 * cos(q(1,0) - q(2,0)));

    fai(2,2) = 0;
    fai(2,3) = 0;

    fai(2,4) = -qd(2,0);
    fai(2,5) = 0;
    fai(2,6) = 0;
    fai(2,7) = -atan2(900*qd(2,0),1)*2.0/(M_PI*1.0);
    fai(2,8) = 0;    
    fai(2,9) = 0;
    fai(2,10) = 1.0;

    fai_dot(0,0) = -1.0 / 3.0 * l1 * l1 *cos(q(1,0))*cos(q(1,0))* dqr_dot(0,0);
    fai_dot(0,1) =  -1.0* ((1.0/ 12.0 * l2 * l2*cos(expect_q(1,0)-expect_q(2,0)) *cos(expect_q(1,0)-expect_q(2,0))  + l1 * l1 * cos(expect_q(1,0)) * cos(expect_q(1,0)) + 1.0/ 4.0 * l2 * l2 * cos(expect_q(1,0) - expect_q(2,0)) * cos(expect_q(1,0) - expect_q(2,0)) + l1 * l2 * cos(expect_q(1,0)) * cos(expect_q(1,0) - expect_q(2,0))) * dqr_dot(0,0) +
        (-2.0 * l1 * l1 * cos(expect_q(1,0)) * sin(expect_q(1,0)) - 2.0 / 3.0 * l2 * l2 * cos(expect_q(1,0) - expect_q(2,0)) * sin(expect_q(1,0) - expect_q(2,0)) - l1 * l2*sin(2*expect_q(1,0) - expect_q(2,0)) ) * expect_qd(1,0) * qr_dot(0,0) +
        (2.0 / 3.0 * l2 * l2 * sin(expect_q(1,0) -expect_q(2,0)) * cos(expect_q(1,0) - expect_q(2,0)) - l1 * l2 * cos(expect_q(1,0)) * sin(expect_q(1,0) - expect_q(2,0))) * expect_qd(2,0) * qr_dot(0,0));
    fai_dot(0,2) = -expect_qd(0,0);
    fai_dot(0,3) = 0;

    fai_dot(0,4) = 0;

    fai_dot(0,5) = -atan2(900*expect_qd(0,0),1)*2.0/(M_PI*1.0);
    fai_dot(0,6) = 0;
    fai_dot(0,7) = 0;

    fai_dot(0,8) = 1.0;    
    fai_dot(0,9) = 0;
    fai_dot(0,10) = 0;

    fai_dot(1,0) = -1.0 * ((1.0 / 3.0 * l1 * l1 * dqr_dot(1,0)) + 1.0 / 2.0 * g * l1 * cos(expect_q(1,0))+1.0/3.0*l1*l1*cos(expect_q(1,0))*sin(expect_q(1,0))*expect_qd(0,0) * qr_dot(0,0));
// df1 df2
    fai_dot(1,1) = -1.0 * ((1.0 / 3.0 * l2 * l2 + l1 * l1 + l1 * l2 * cos(expect_q(2,0))) * dqr_dot(1,0) -  l1 * l2 * sin(expect_q(2,0)) * expect_qd(2,0) * qr_dot(1,0) + (-1.0 / 3.0 * l2 * l2 - 0.5 * l1 * l2 * cos(expect_q(2,0))) * dqr_dot(2,0) +
        0.5 * l1 * l2 * sin(expect_q(2,0)) * expect_qd(2,0) * qr_dot(2,0) +(l1 * l1 * cos(expect_q(1,0)) * sin(expect_q(1,0)) + 0.5*l1 * l2 * sin(2 * expect_q(1,0) - expect_q(2,0)) + 1.0/3.0* l2 * l2 * cos(expect_q(1,0) - expect_q(2,0)) * sin(expect_q(1,0) - expect_q(2,0))) * expect_qd(0,0) * qr_dot(0,0) +
        g * l1 * cos(expect_q(1,0))  + 0.5 * g * l2 * cos(expect_q(1,0) - expect_q(2,0)));

    /*fai_dot(1,0) = -1.0 * ((1.0 / 3.0 * l1 * l1 * dqr(1,0)) + 1.0 / 2.0 * g * l1 * cos(q(1,0)));
// df1 df2
    fai_dot(1,1) = -1.0 * ((1.0 / 3.0 * l2 * l2 + l1 * l1 + l1 * l2 * cos(q(2,0))) * dqr(1,0) -  l1 * l2 * sin(q(2,0)) * qd(2,0) * qr(1,0) + (1.0 / 3.0 * l2 * l2 + 0.5 * l1 * l2 * cos(q(2,0))) * dqr(2,0) -
        0.5 * l1 * l2 * sin(q(2,0)) * qd(2,0) * qr(2,0) + (l1 * l1 * cos(q(1,0)) * sin(q(1,0)) + l1 * l2 * sin(2 * q(1,0) + q(2,0)) + 0.25 * l2 * l2 * cos(q(1,0) + q(2,0)) * sin(q(1,0) + q(2,0))) * qd(0,0) * qr(0,0) +
        g * l1 * cos(q(1,0))  + 0.5 * g * l2 * cos(q(1,0) + q(2,0)));*/

    fai_dot(1,2) = 0;
    fai_dot(1,3) = -expect_qd(1,0);
    fai_dot(1,4) = 0;

    fai_dot(1,5) = 0;
    fai_dot(1,6) = -atan2(900*expect_qd(1,0),1)*2.0/(M_PI*1.0);
    fai_dot(1,7) = 0;

    fai_dot(1,8) = 0;    
    fai_dot(1,9) = 1;
    fai_dot(1,10) = 0;
    
    fai_dot(2,0) = 0; 
    fai_dot(2,1) = -1 * (1.0 / 3.0 * l2 * l2 * dqr_dot(2,0) + (-1.0 / 3.0 * l2 * l2 - 0.5 * l1 * l2 * cos(expect_q((2,0)))) * dqr_dot(1,0) +
       ( 1.0/3.0* l2 * l2 * cos(expect_q(1,0) - expect_q(2,0)) * sin(expect_q(1,0) - expect_q(2,0)) + 0.5 * l1 * l2 * cos(expect_q(1,0)) * sin(expect_q(1,0) - expect_q(2,0)) )* expect_qd(0,0) * qr_dot(0,0) +
        0.5 * l1 * l2 * sin(expect_q(2,0)) * expect_qd(1,0) * qr_dot(1,0) - 0.5 * g * l2 * cos(expect_q(1,0) - expect_q(2,0)));
    fai_dot(2,2) = 0;
    fai_dot(2,3) = 0;
    fai_dot(2,4) = -expect_qd(2,0);
    fai_dot(2,5) = 0;
    fai_dot(2,6) = 0;
    fai_dot(2,7) = -atan2(900*expect_qd(2,0),1)*2.0/(M_PI*1.0);
    fai_dot(2,8) = 0;    
    fai_dot(2,9) = 0;
    fai_dot(2,10) = 1;
  //  if(abs(expect_qd(1,0))<=0.001&&trajectory_flag==1)
    //        gammamamm(9,9) = 15000;











    theta_d = gammamamm*fai.transpose()*s*t;
  /*  for(int i=0;i<11;i++)
    {
        theta_d(i,0) = gammamamm(i,i)*(fai(0,i)*s(0,0)+fai(1,i)*s(1,0)+fai(1,i)*s(2,0))*t;
        if(i==0)
           {
                ROS_INFO("m1= %lf,theta_d = %lf",theta[0],theta_d[0]);
                ROS_INFO("fai= %lf,fai(0,i) = %lf,fai(1,i) = %lf,",fai(0,i),fai(1,i),fai(2,i));
                ROS_INFO("s(1,0)= %lf,s(1,0) = %lf,s(2,0)= %lf,",s(0,0),s(1,0),s(2,0));
           }
           
    }*/
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

    us1 = -theta_max1*k1*s;
    ua = -1.0*fai_dot*theta;
  
    u = us1+ua;
    return u;

}


Vector3d Kinematics_Solver(double q1,double q2,double q3,double KB_W_,double KB_A_,double KB_Q_,double KB_SPEEDUP_,double SPEEDUP1,double SPEEDUP2,double SPEEDUP3)
{
    double a1 = 424.0;
    //double a2 = 268.0;
    double a2 = 325.0;
    double update_x;
    double update_y;
    double update_z;
    double actual_x;
    double actual_y;
    double actual_z;
    double actual_r ; 
    double W;
    double A;
    double theta_T;
    double theta1;
    double theta2;
    double theta3;  
    double upd_disx=0.48;
    double upd_dis=0.48;
    double N_1;
    double N_2;
    double N_3;
    double N_4;
    double KB_W_Kinematics;
    double KB_A_Kinematics;
    double KB_Q_Kinematics;
    double KB_SPEEDUP_coefficient;
    double upd_dis1,upd_dis2,upd_dis3;
    KB_W_Kinematics=KB_W_;
    KB_A_Kinematics=KB_A_;
    KB_Q_Kinematics=KB_Q_;
    KB_SPEEDUP_coefficient=KB_SPEEDUP_;
    upd_dis1=abs(SPEEDUP1)*upd_disx;
    upd_dis2=abs(SPEEDUP2)*upd_disx;
    upd_dis3=abs(SPEEDUP3)*upd_disx;   

    //正运动学求解
    actual_r = a1*cos(-q2)+a2*cos(-q2+q3);
    actual_x=actual_r*cos(q1);
    actual_y=actual_r*sin(q1);
    actual_z=a1*sin(-q2)+a2*sin(-q2+q3);

    upd_dis=KB_SPEEDUP_coefficient*upd_disx;



    // 满足工作空间内的笛卡尔空间更新
    if(abs(KB_W_Kinematics+1)<0.1&&((actual_x-upd_dis)*(actual_x-upd_dis)+actual_y*actual_y+actual_z*actual_z)<=561001&&((actual_x-upd_dis)*(actual_x-upd_dis)+actual_y*actual_y+actual_z*actual_z)>=9801)
    //if(abs(KB_W_Kinematics+1)<0.1)
    {
        update_x=actual_x-upd_dis1;
    }
    else  if(abs(KB_W_Kinematics-1)<0.1&&((actual_x+upd_dis)*(actual_x+upd_dis)+actual_y*actual_y+actual_z*actual_z)<=561001&&((actual_x+upd_dis)*(actual_x+upd_dis)+actual_y*actual_y+actual_z*actual_z)>=9801)
    //else  if(abs(KB_W_Kinematics-1)<0.1)
    {
        update_x=actual_x+upd_dis1;
    }
    else{
        update_x=actual_x;
    }

    if(abs(KB_A_Kinematics+1)<0.1&&((actual_x)*(actual_x)+(actual_y+upd_dis)*(actual_y+upd_dis)+actual_z*actual_z)<=561001&&((actual_x)*(actual_x)+(actual_y+upd_dis)*(actual_y+upd_dis)+actual_z*actual_z)>=9801)
    //if(abs(KB_W_Kinematics+1)<0.1)
    {
        update_y=actual_y+upd_dis2;
    }
    else  if(abs(KB_A_Kinematics-1)<0.1&&((actual_x)*(actual_x)+(actual_y-upd_dis)*(actual_y-upd_dis)+actual_z*actual_z)<=475610018864&&((actual_x)*(actual_x)+(actual_y-upd_dis)*(actual_y-upd_dis)+actual_z*actual_z)>=9801)
    //else  if(abs(KB_W_Kinematics-1)<0.1)
    {
        update_y=actual_y-upd_dis2;
    }
    else{
        update_y=actual_y; 
    }

    if(abs(KB_Q_Kinematics+1)<0.1&&((actual_x)*(actual_x)+(actual_y)*(actual_y)+(actual_z-upd_dis)*(actual_z-upd_dis))<=561001&&((actual_x)*(actual_x)+(actual_y)*(actual_y)+(actual_z-upd_dis)*(actual_z-upd_dis))>=9801)
    //if(abs(KB_W_Kinematics+1)<0.1)
    {
        update_z=actual_z-upd_dis3;
    }
    else  if(abs(KB_Q_Kinematics-1)<0.1&&((actual_x)*(actual_x)+(actual_y)*(actual_y)+(actual_z+upd_dis)*(actual_z+upd_dis))<=561001&&((actual_x)*(actual_x)+(actual_y)*(actual_y)+(actual_z+upd_dis)*(actual_z+upd_dis))>=9801)
    //else  if(abs(KB_W_Kinematics-1)<0.1)
    {
        update_z=actual_z+upd_dis3;
    }
    else{
        update_z=actual_z;
    }


    // 逆运动学求解
    W=sqrt(update_x*update_x+update_y*update_y);
    A=sqrt(update_x*update_x+update_y*update_y+update_z*update_z);

    N_1=update_z/A;
    if(N_1>1.00){
        N_1=1.00;
    }
    else if(N_1<-1.00){
        N_1=-1.00;
    }

    N_2=update_y/W;
    if(N_2>1.00){
        N_2=1.00;
    }
    else if(N_2<-1.00){
        N_2=-1.00;
    }

    N_3=(a1*a1+A*A-a2*a2)/(2*a1*A);
    if(N_3>1.00){
        N_3=1.00;
    }
    else if(N_3<-1.00){
        N_3=-1.00;
    }

    N_4=(A*A-a1*a1-a2*a2)/(2*a1*a2);
    if(N_4>1.00){
        N_4=1.00;
    }
    else if(N_4<-1.00){
        N_4=-1.00;
    }

    //theta_T=asin(update_z/A);
    theta_T=asin(N_1);

    //theta1=asin(update_y/W);
    //theta1=asin(N_2);

    theta1=atan2(update_y,update_x);
    if(theta1>=PI*9/12||theta1<=-PI/7 )
    //if(theta1>PI/2||theta1<0)
    {
        theta1=q1;
        theta2=q2;
        theta3=q3;
    }
    else{
           //theta2=(-1.0)*acos((a1*a1+A*A-a2*a2)/(2*a1*A))+theta_T;
        theta2=(-1.0)*acos(N_3)+theta_T;
        //theta3=(1.0)*acos((A*A-a1*a1-a2*a2)/(2*a1*a2));
        theta3=(1.0)*acos(N_4);     
    }
    Kinematics_theta_update(0,0)=theta1;
    Kinematics_theta_update(1,0)=-theta2;
    Kinematics_theta_update(2,0)=theta3;
    //ROS_INFO("xyz===  %f , %f , %f ,//, %f , %f , %f ",actual_x,actual_y,actual_z,update_x,update_y,update_z);
    //ROS_INFO("q1q2q3===  %f , %f , %f ,//, %f , %f , %f ",q1,q2,q3,theta1,-theta2,theta3);
    //ROS_INFO("JOY_WAQ===  %f , %f ,%f  ",KB_W,KB_A,KB_Q);
    return Kinematics_theta_update;
}