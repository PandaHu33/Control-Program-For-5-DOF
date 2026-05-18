#include "mainpulator/mavlink_udp.h"
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */
#include<stdint.h>

//随机数
#include<stdlib.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#define N_rand 99

    struct sockaddr_in gcAddr;
    struct sockaddr_in locAddr;
    int sock = 0;
    uint8_t systemId = SYSTEM_ID;

    char target_ip[100];
    float position[6] = {10.1, 23.7, 34.5, 3.2, 1.1, 3.3};

    // 定义发送和接收数据包的缓存区
    uint8_t buf[BUFFER_LENGTH];
    uint8_t new_buf[BUFFER_LENGTH];
    ssize_t recsize;
    socklen_t fromlen = sizeof(gcAddr);
    int bytes_sent;

    // 定义 MAVLink 消息结构体和消息长度
    mavlink_message_t msg;
    uint16_t len;

    // 定义循环计数器
    int i = 0;
    char help[] = "--help";
    //float tmp_depth_target;

/*
void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set);
void handle_heartbeat(const mavlink_message_t* message);
void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
*/

double MVLink_udp_expect_q1=4.0001;
double MVLink_udp_expect_q2=5.0001;
double MVLink_udp_expect_q3=6.0001;
double MVLink_udp_expect_q5=0.0001;
double MVLink_udp_actual_q1 = 1.0001;
double MVLink_udp_actual_q2 = 2.0001;
double MVLink_udp_actual_q3 = 3.0001;
double MVLink_udp_actual_q5 = 0.0001;

//上传到上位机的参数
double PU_q1=0.0;
double PU_q2=0.0;
double PU_q3=0.0;
double PU_q4=0.0;
double PU_q5 = 0.0;
double PU_qd1 = 0.0;
double PU_qd2 = 0.0;
double PU_qd3 = 0.0;
double PU_qd4 = 0.0;
double PU_Current5= 0.0;

double PU_K_x=0.0;
double PU_K_y=0.0;
double PU_K_z=0.0;
double PU_Buttom=0.0;
double PU_status=0.0;

//来自上位机的数据
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
double RE_Buttom=0.0;
double RE_Flag1=0.0;
double RE_Flag2=0.0;
double RE_Flag3=0.0;
void AngleCallback(const sensor_msgs::JointState& receive_message);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavlink_udp_node");
    ros::NodeHandle udp_send;

    ros::NodeHandle  angle_receive;
	ros::Subscriber  angle_sub =angle_receive.subscribe("Angle_Pub_To_All_Nodes",100,AngleCallback);

    ros::NodeHandle MVLink_pub;
    ros::Publisher MVLink_Pub_To_Main_Node = MVLink_pub.advertise<sensor_msgs::JointState>("MVLink_Pub_To_Main_Node",100);  

    ros::Rate loop_rate(100); // while以2Hz循环

    strcpy(target_ip, CLIENT_IP);

    // 1.创建一个通信的socket
    int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    //ROS_INFO("Try Mavlink socket");
    if(sock == -1) {
        perror("socket");
        exit(-1);
    }   

    // Server Address config
    // 配置服务器地址
    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(SERVER_PORT);
    // 2.绑定
    int ret = bind(sock, (struct sockaddr *)&locAddr, sizeof(struct sockaddr));
    if(ret == -1) {
        perror("[ERR] bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

     // 配置客户端地址
    // Client Address config
    memset(&gcAddr, 0, sizeof(gcAddr));
    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = inet_addr(target_ip);
    gcAddr.sin_port = htons(CLIENT_PORT);

    // 输出提示信息
    printf("Start sending/receiving MAVLink message to/from QGroundControl...\n");

    
    while (ros::ok)
    {
        // 发送心跳消息
        mavlink_msg_heartbeat_pack(systemId, COMPONENT_ID, &msg,
                                   MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr *)&gcAddr, sizeof(struct sockaddr));
	    //ROS_INFO("HerartBeat: send!--");

/*
        //mavlink_msg_altitude_pack(1, 1, &msg, 12345, 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
        mavlink_msg_attitude_pack(1, 1, &msg, 12345, MVLink_udp_actual_q1, MVLink_udp_actual_q2, MVLink_udp_actual_q3, MVLink_udp_expect_q1, MVLink_udp_expect_q2,MVLink_udp_expect_q3);
        len = mavlink_msg_to_send_buffer(buf, &msg);        
        bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr *)&gcAddr, sizeof(struct sockaddr));
        ROS_INFO("Altitude: send!---");
*/

        mavlink_msg_manipulator2qgc_pack(1, 1, &msg,PU_q1,PU_q2,PU_q3,PU_q4,PU_q5,PU_qd1,PU_qd2,PU_qd3,PU_qd4,PU_Current5,
                                                                                                    PU_K_x,PU_K_y,PU_K_z,PU_Buttom,1,2,3,4,5,6,7,8,9,10,PU_status,888,1,2,3,4);
        len = mavlink_msg_to_send_buffer(buf, &msg);        
        bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr *)&gcAddr, sizeof(struct sockaddr));
        //ROS_INFO("MSGinfo: send!---");

        memset(buf, 0, BUFFER_LENGTH);// 使用memset将buf内存区域的每个字节设置为0


        //设置阻塞超时
        struct timeval timeOut;
        timeOut.tv_sec = 0;                 //设置5s超时
        timeOut.tv_usec = 1*1000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeOut, sizeof(timeOut));

        int recsize;
        int recsize_count;
        int count_=0;
        while((recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen))!= -1)
        {
            memcpy(new_buf,buf,sizeof(buf)); //小心内存溢出
            recsize_count=recsize;
            count_++;
        }
        if(count_!=0)
        {
            memcpy(buf,new_buf,sizeof(new_buf)); //小心内存溢出
            recsize=recsize_count;
        }
        else
        {
            recsize=0;
        }

        if(recsize > 0)
		{
            
			//printf("---------------------Bytes Received: %d\nDatagram: ", (int)recsize);
	        for(i = 0; i < recsize; i++)
			{
                //使用printf函数和%02x的格式化选项将每个字节打印为两位的十六进制数。
				printf("%d ", (unsigned char)buf[i]);
            }
            //printf("\n*************\n");
            

           	mavlink_message_t msg;
			mavlink_status_t status; 
            for(i = 0; i < recsize; i++)
            {
                if(mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status) == 1)
                {
                    //printf("id===%d\n",msg.msgid);
                    switch (msg.msgid)
                    {
                        case MAVLINK_MSG_ID_HEARTBEAT:
                            //printf("heartbeat::\n");
                            mavlink_heartbeat_t heartbeat;
                            mavlink_msg_heartbeat_decode(&msg,&heartbeat);
                            //printf("type: %x\n",(int)heartbeat.type);
                            //printf("autopilot: %x\n",(int)heartbeat.autopilot);
                            //printf("base_mode: %x\n",(int)heartbeat.base_mode);
                            //printf("custom_mode: %x\n",(int)heartbeat.custom_mode);
                            //printf("mavlink_version: %x\n",(int)heartbeat.mavlink_version); 
                            //printf("system_status: %x\n",(int)heartbeat.system_status);
                            break;
                          /* 
                        case MAVLINK_MSG_ID_ATTITUDE:
                            printf("attitude::\n");
                            mavlink_attitude_t attitude;
                            mavlink_msg_attitude_decode(&msg,&attitude);
                            printf("a: %x\n",(int)attitude.roll);
                            printf("b: %x\n",(int)attitude.pitch);
                            printf("c: %x\n",(int)attitude.yaw);
                            printf("d: %x\n",(int)attitude.rollspeed);
                            printf("e: %x\n",(int)attitude.pitchspeed);
                            printf("f: %x\n",(int)attitude.yawspeed);                  
                            break;
                         */    
                        case MAVLINK_MSG_ID_QGC2MANIPULATOR:
                            //printf("QGC2MPinfo::\n");
                            mavlink_qgc2manipulator_t QGC2MPinfo;
                            mavlink_msg_qgc2manipulator_decode(&msg,&QGC2MPinfo);
                            RE_q1=QGC2MPinfo.angle1;
                            RE_q2=QGC2MPinfo.angle2;
                            RE_q3=QGC2MPinfo.angle3;
                            RE_q4=QGC2MPinfo.angle4;    
                            RE_q5=QGC2MPinfo.angle5;
                            RE_qd1=QGC2MPinfo.angular_vel1;              
                            RE_qd2=QGC2MPinfo.angular_vel2;    
                            RE_qd3=QGC2MPinfo.angular_vel3;    
                            RE_qd4=QGC2MPinfo.angular_vel4;    
                            RE_Current5=QGC2MPinfo.angular_vel5; 

                            RE_KB_W=QGC2MPinfo.keyboard1;
                            RE_KB_A=QGC2MPinfo.keyboard2;
                            RE_KB_S=QGC2MPinfo.keyboard3;
                            RE_KB_D=QGC2MPinfo.keyboard4;
                            RE_KB_Q=QGC2MPinfo.keyboard5;
                            RE_KB_E=QGC2MPinfo.keyboard6;

                            RE_KB_O=QGC2MPinfo.keyboard7;
                            RE_KB_C=QGC2MPinfo.keyboard8;
                            RE_KB_R=QGC2MPinfo.keyboard9;
                            RE_KB_L=QGC2MPinfo.keyboard10;
                            RE_status=QGC2MPinfo.status;
                            RE_Buttom=QGC2MPinfo.button;
                            RE_Flag1=QGC2MPinfo.pos_x;
                            //printf("kb1: %f\n",RE_KB_W);
                            //printf("kb2: %f\n",RE_KB_A);
                            //printf("omni: %f\n",RE_Flag1);        
                            //printf("RE_status: %f\n",RE_status);
                            //printf("RE_q1:%f\n",RE_q1);
                            //printf("RE_q2:%f\n",RE_q2);
                            //printf("RE_q3:%f\n",RE_q3);
                            //printf("RE_Flag1:%f\n",RE_Flag1 );
                            break;


                    }
                }
            }
        }
        memset(buf, 0, BUFFER_LENGTH);// 使用memset将buf内存区域的每个字节设置为0


        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.position.resize(8);
        joint_state.velocity.resize(30);
        joint_state.velocity[0] = RE_q1;
        joint_state.velocity[1] = RE_q2 ;
        joint_state.velocity[2] = RE_q3 ;
        joint_state.velocity[3] = RE_q4;
        joint_state.velocity[4] = RE_q5;

        joint_state.velocity[5] = RE_qd1;
        joint_state.velocity[6] = RE_qd2 ;
        joint_state.velocity[7] = RE_qd3 ;
        joint_state.velocity[8] = RE_qd4;
        joint_state.velocity[9] = RE_Current5;

        joint_state.velocity[10] = RE_KB_W;
        joint_state.velocity[11] = RE_KB_A ;
        joint_state.velocity[12] = RE_KB_S ;
        joint_state.velocity[13] = RE_KB_D;
        joint_state.velocity[14] = RE_KB_Q;
        joint_state.velocity[15] = RE_KB_E;

        joint_state.velocity[16] = RE_KB_O;
        joint_state.velocity[17] = RE_KB_C;
        joint_state.velocity[18] = RE_KB_R;
        joint_state.velocity[19] = RE_KB_L;

        joint_state.velocity[20] = RE_status;
        joint_state.velocity[21] = RE_Buttom;
        joint_state.velocity[22] = RE_Flag1;
        joint_state.velocity[23] = RE_Flag2;
        MVLink_Pub_To_Main_Node.publish(joint_state);


        ros::spinOnce();
        loop_rate.sleep();
	}
	return 0;
}

void AngleCallback(const sensor_msgs::JointState& receive_message)
{
    float random;
    srand(time(NULL));
     random =rand() % (N_rand+1) / (float)(N_rand+1);
        PU_q1=receive_message.velocity[0];
        PU_q2=receive_message.velocity[1];
        PU_q3=receive_message.velocity[2];
        PU_q4=receive_message.velocity[3];
        PU_q5 = receive_message.velocity[4];
        PU_qd1 = receive_message.velocity[5];
        PU_qd2 = receive_message.velocity[6];
        PU_qd3 = receive_message.velocity[7];
        PU_qd4 = receive_message.velocity[8];
        PU_Current5= receive_message.velocity[9];
        PU_K_x= receive_message.velocity[10];
        PU_K_y= receive_message.velocity[11];
        PU_K_z= receive_message.velocity[12];
        PU_status= receive_message.velocity[13];

	    //actual_q5 = receive_message.position[];
        //ROS_INFO("actual_q123 = %f,%f,%f", MVLink_udp_actual_q1,MVLink_udp_actual_q2,MVLink_udp_actual_q3);
        //ROS_INFO("expect_q123 = %f,%f,%f", MVLink_udp_expect_q1,MVLink_udp_expect_q2,MVLink_udp_expect_q3);
}

/*
void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set)
{
    // We just receive one UDP datagram and then return again.
    char buffer[2048]; // enough for MTU 1500 bytes

    const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(src_addr), src_addr_len);

    if (ret < 0) {
        printf("recvfrom error: %s\n", strerror(errno));
    } else if (ret == 0) {
        // peer has done an orderly shutdown
        return;
    } 

    *src_addr_set = true;

    mavlink_message_t message;
    mavlink_status_t status;
    for (int i = 0; i < ret; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {

            // printf(
            //     "Received message %d from %d/%d\n",
            //     message.msgid, message.sysid, message.compid);

            switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                handle_heartbeat(&message);
                break;
            }
        }
    }
}

void handle_heartbeat(const mavlink_message_t* message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message, &heartbeat);

    printf("Got heartbeat from ");
    switch (heartbeat.autopilot) {
        case MAV_AUTOPILOT_GENERIC:
            printf("generic");
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            printf("ArduPilot");
            break;
        case MAV_AUTOPILOT_PX4:
            printf("PX4");
            break;
        default:
            printf("other");
            break;
    }
    printf(" autopilot\n");
}

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // Whenever a second has passed, we send a heartbeat.
    static time_t last_time = 0;
    time_t current_time = time(NULL);
    if (current_time - last_time >= 1) {
        send_heartbeat(socket_fd, src_addr, src_addr_len);
        last_time = current_time;
    }
}

void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    const uint8_t system_id = 42;
    const uint8_t base_mode = 0;
    const uint8_t custom_mode = 0;
    mavlink_msg_heartbeat_pack_chan(
        system_id,
        MAV_COMP_ID_PERIPHERAL,
        MAVLINK_COMM_0,
        &message,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        base_mode,
        custom_mode,
        MAV_STATE_STANDBY);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Sent heartbeat\n");
    }
}
*/