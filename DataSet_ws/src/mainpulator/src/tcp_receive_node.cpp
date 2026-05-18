#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include "mainpulator/MyTcpip.h"
#define PORT 9000 //端口号
#define LOG 1	  //请求队列中最大连接数

using namespace std;

vector<double> jointActualAngles = {0, 0, 0, 0}, endPose;

void callback(const sensor_msgs::JointState &joint_state)
{
	for (int i = 0; i < 3; i++)
	{

		jointActualAngles[i] = joint_state.position[i] / 3.14 * 180;
	}
	jointActualAngles[3] = 0;
	//ROS_INFO("mainpulator joint expectangle  %lf   %lf   %lf",jointActualAngles[0],jointActualAngles[1],jointActualAngles[2]);
}
char msg_char[80];
int main(int argc, char **argv)
{
	ros::init(argc, argv, "tcp_receive_node");
	//ros::NodeHandle nh;
	//ros::NodeHandle mainpulator;
	ros::NodeHandle serversend;
	/*
	 *@fuc: 监听套节字描述符和连接套节字描述符
	 *@fuc; 服务器端和客户端IP4地址信息,struct关键字可不要
	 */
	int listenfd, connectfd;
	struct sockaddr_in sever;
	struct sockaddr_in client;
	socklen_t addrlen;

	/*
	 *@fuc: 使用socket()函数产生套节字描述符
	 */
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	if (listenfd == -1)
	{
		printf("socket() error\n");
		return -1;
	}

	/*
	 *@fuc: 初始化server套节字地址信息
	 */
	memset((void *)&sever, 0, sizeof(sever));
	sever.sin_family = AF_INET;
	sever.sin_addr.s_addr = htonl(INADDR_ANY);
	sever.sin_port = htons(PORT);

	/*
	 *@fuc: 用bind()函数，将套接字与指定的协议地址绑定
	 */
	if (bind(listenfd, (struct sockaddr *)&sever, sizeof(sever)) < 0)
	{
		printf("bind() error\n");
		return -1;
	}

	/*
	 *@fuc: 使用listen()函数，等待客户端的连接
	 */
	if (listen(listenfd, LOG) < 0)
	{
		printf("listen() error.\n");
		return -1;
	}

	addrlen = sizeof(client);
	ros::Rate loop_rate(500); // while以50Hz循环
	const int BUFF_SIZE = 80;
	char buf[BUFF_SIZE];
	int size_recv;
	//不断监听客户端请求
	connectfd = accept(listenfd, (struct sockaddr *)&client, &addrlen);

	//ros::Subscriber mpul_sub_ = mainpulator.subscribe("test_node/chatter",10,callback);
	ros::Publisher pub = serversend.advertise<sensor_msgs::JointState>("servermessage", 100);

	std::vector<double> rcvData;
	rcvData.resize(10);

	while (ros::ok)
	{
		// accept
		// connectfd = accept(listenfd,(struct sockaddr *)&client,&addrlen);
		if (connectfd < 0)
		{
			printf("connect() error \n");
			return -1;
		}
		// printf("You got a connection from client's IP is %s, port is %d\n",
		//		inet_ntoa(client.sin_addr), ntohs(client.sin_port));

		//msgMotorTemplate motorData = msgMotorTemplate(123, 0, jointActualAngles, 0, {0, 0, 0, 0, 0}, "Motor data for test!");
		//display(motorData);
		//send(connectfd, "1111",4,0);

		size_recv = recv(connectfd, buf, 80, 0);
		if (size_recv <= 0)
		{
			continue;
		}
		else
		{
			buf[size_recv] = '\0';
			// for(int i=0; i<size_recv; i++){
			//	cout<<buf[i];
			// }
			// cout<<endl;
			//if (size_recv >256)
			//{
			//	cout<<"receive_length:" <<size_recv<<endl;
			//}

			doubletochar_8 data_element;
			if (size_recv % 80 != 0)
			{
				cout << "Received message has a wrong length (supposed to be 256)" << endl;
				cout << "receive_length:" << size_recv << endl;
			}
			else
			{
				for (int j = 0; j < 10; j++)
				{
					for (int k = 0; k < 8; k++)
					{
						data_element.num_char[k] = buf[8 * j + 7 - k];
					}
					rcvData[j] = data_element.num_double;
				}
			}
		}
		sensor_msgs::JointState joint_state;
		joint_state.header.stamp = ros::Time::now();
		joint_state.position.resize(5);
		joint_state.velocity.resize(5);
		joint_state.effort.resize(5);
		//joint_state.position[0] = expect_q;
		//joint_state.position[1] = joint_1.get_current_angle();

		joint_state.position[0] = rcvData[0];
		joint_state.position[1] = rcvData[1];
		joint_state.position[2] = rcvData[2];
		joint_state.velocity[0] = rcvData[3];
		joint_state.velocity[1] = rcvData[4];
		joint_state.velocity[2] = rcvData[5];
		joint_state.effort[0] = rcvData[6];
		joint_state.effort[1] = rcvData[7];
		joint_state.effort[2] = rcvData[8];
		joint_state.position[3] = rcvData[9];
		//joint_state.position[4] = (double)stdMsgCamera(msg_char).msg.state;

		ROS_INFO("mainpulator joint recv  %lf   %lf   %lf  %lf", joint_state.position[0], joint_state.position[1], joint_state.position[2], joint_state.position[3]);

		//ROS_INFO("mainpulator joint currentangle  STATE %d  ",stdMsgCamera(msg_char).msg.state);
		//ROS_INFO("mainpulator joint currentangle  STATE %lf   ",joint_state.position[4]);

		pub.publish(joint_state);

		// send(connectfd, "1111",4,0);
		// close(connectfd);
		ros::spinOnce();
		loop_rate.sleep();
		// loop_rate.sleep();
	}
	close(connectfd);
	close(listenfd);
	return 0;
}
