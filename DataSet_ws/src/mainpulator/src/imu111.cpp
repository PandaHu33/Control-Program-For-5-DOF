#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <stdlib.h>
#include<iostream>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>

#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define PI acos(-1)
#define time 0.01;
using namespace std;
using namespace Eigen;
double R,P,Y;
double pitch;
double roll;
double yaw;

void euler_angles_Callback(const geometry_msgs::Vector3 & msg);

const int PORT = 8000;
int serverSocket, clientSocket;
struct sockaddr_in serverAddress, clientAddress;
socklen_t clientAddressLength = sizeof(clientAddress);

int serverBuild()
{
    // 创建服务器套接字
    if ((serverSocket = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "Failed to create socket." << std::endl;
        exit(EXIT_FAILURE);
    }

    // 设置服务器地址结构
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(PORT);

    // 绑定服务器套接字到指定端口
    if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Failed to bind socket." << std::endl;
        exit(EXIT_FAILURE);
    }
}

void serverConnect(){
    // 监听连接请求
    if (listen(serverSocket, 3) < 0) {
        std::cerr << "Failed to listen." << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << PORT << std::endl;

    // 接受客户端连接
    if ((clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddress, &clientAddressLength)) < 0) {
        std::cerr << "Failed to accept connection." << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Client connected." << std::endl;
}

bool serverSend(double* dataArray, int length)
{
    //double dataArray[] = {1.23, 4.56, 7.89}; // 要发送的double数组
    // 发送数据给客户端
    //if (send(clientSocket, reinterpret_cast<char *>(dataArray), sizeof(dataArray), 0) < 0)
    if (send(clientSocket, reinterpret_cast<char *>(dataArray), sizeof(double) * length, 0) < 0) {
        std::cerr << "Failed to send data." << std::endl;
		return false;
        exit(EXIT_FAILURE);
    }
    std::cout << "Data sent successfully." << std::endl;
	return true;
}

void serverClose()
{
    // 关闭套接字
    close(clientSocket);
    close(serverSocket);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Imu_Euler_Node");
	ros::NodeHandle n;
	ros::Publisher Imu_Info_pub = n.advertise<sensor_msgs::JointState>("Imu_Info_", 100);

	ros::NodeHandle Imu_Euler;
    ros::Subscriber Imu_Euler_Sub=Imu_Euler.subscribe("/euler_angles",10,euler_angles_Callback);

	serverBuild();
	serverConnect();

	ros::Rate loop_rate(500);
    while (ros::ok)
    {

 		sensor_msgs::JointState Imu;
        Imu.header.stamp = ros::Time::now();
        Imu.position.resize(8);
        Imu.velocity.resize(8);
        Imu.effort.resize(8);
        Imu.position[0] = R;
        Imu.position[1] = P;
        Imu.position[2] = Y;
        Imu_Info_pub.publish(Imu);		
		if(!serverSend(&Imu.position[0], 3)){
			cout<<"error"<<endl;
		}
		//ROS_INFO("imuR= %lf,%lf,%lf,%lf,%lf,%lf",camera_R,imu_m(0), camera_P,imu_m(1),camera_Y,imu_m(2));
		ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}





void euler_angles_Callback(const  geometry_msgs::Vector3 & msg)
{
    R = msg.x;
    P = msg.y;
    Y = msg.z; 
	if(Y<0)
	{
		Y=Y+2*PI;
	}
	else if(Y>2*PI)
	{
		Y=Y-2*PI;
	}

	if(Y>PI)
	{
		Y=Y-2*PI;
	}
	R=(R/PI)*180;
	P=(P/PI)*180;
	Y=(Y/PI)*180;
}



