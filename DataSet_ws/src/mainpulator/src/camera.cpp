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
#include<geometry_msgs/Point.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include  "mainpulator/MyTcpip.h"
#define PORT 9000 //端口号
#define LOG 1	  //请求队列中最大连接数

using namespace std;

void display(msgCameraTemplate msg)
{
	cout << endl;
	std::cout << "------------Standard Camera Data-------------" << std::endl;
	//std::cout << "Message length: " << msg.length << std::endl;
	std::cout << "Message ID: " << msg.id << std::endl;
	//std::cout << "Vision program state: " << msg.state << std::endl;
	std::cout << "Joint desired angles: ";
	for (int i = 0; i < 4; i++)cout << msg.jointDesiredAngles[i] << " ";
	cout << std::endl;
	//std::cout << "Clamper desired space: " << msg.clamperDesiredSpace << std::endl;
	//std::cout << "Target pose relative to camera: ";
	//for (int i = 0; i < 6; i++)cout << msg.targetRelativePose[i] << " ";
	//cout << std::endl;
	//std::cout << "Target global position: ";
	//for (int i = 0; i < 3; i++)cout << msg.targetGlobalPosition[i] << " ";
	//cout << std::endl;
	//std::cout << "Objective point positioin: ";
	//for (int i = 0; i < 3; i++)cout << msg.objectivePointGlobalPosition[i] << " ";
	//cout << std::endl;
	//std::cout << "Note: " << msg.noteStr << std::endl;
	std::cout << "-------------------------------------------" << std::endl;
}

void display(msgMotorTemplate msg)
{
	cout << endl;
	std::cout << "--------------Standard Motor Data֡-------------" << std::endl;
	std::cout << "Message length: " << msg.length << std::endl;
	std::cout << "Message ID: " << msg.id << std::endl;
	std::cout << "Control program state: " << msg.state << std::endl;
	std::cout << "Joint actual angles: ";
	for (int i = 0; i < 4; i++)cout << msg.jointActualAngles[i] << " ";
	cout << std::endl;
	std::cout << "Clamper actual space: " << msg.clamperActualSpace << std::endl;
	std::cout << "Motor velocities: ";
	for (int i = 0; i < 5; i++)cout << msg.motorVelocity[i] << " ";
	cout << std::endl;
	std::cout << "Note: " << msg.noteStr << std::endl;
	std::cout << "-------------------------------------------" << std::endl;
}
vector<double> jointActualAngles = {0, 0, 0, 0};
void callback(const sensor_msgs::JointState& joint_state)
{

    for(int i=0;i<3;i++)
    {

        jointActualAngles[i]  = joint_state.position[i]/3.14*180;
		
    }    
    jointActualAngles[3] = 0;
	ROS_INFO("mainpulator joint expectangle  %lf   %lf   %lf",jointActualAngles[0],jointActualAngles[1],jointActualAngles[2]);

}
char msg_char[256];
int main(int argc, char **argv)
{
	ros::init(argc, argv, "server_node");

	//ros::NodeHandle nh;
    ros::NodeHandle mainpulator;
	//zhushi
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
		printf("socket() success\n");
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
		printf("bind() success\n");

	/*
	 *@fuc: 使用listen()函数，等待客户端的连接
	 */
	if (listen(listenfd, LOG) < 0)
	{
		printf("listen() error.\n");
		return -1;
	}
		printf("listen() success\n");

	addrlen = sizeof(client);
	ros::Rate loop_rate(500); // while以50Hz循环
	const int BUFF_SIZE = 4096;
	char buf[BUFF_SIZE];
	int size_recv;
	//不断监听客户端请求
	connectfd = accept(listenfd, (struct sockaddr *)&client, &addrlen);




	//	话题订阅主节点，tcp发视觉
    ros::Subscriber mpul_sub_ = mainpulator.subscribe("test_node/chatter",10,callback);
	//zhushi
    ros::Publisher pub =  serversend.advertise<sensor_msgs::JointState>("servermessage",100);
	int i_count=0;		
	int msg_tocamera_count =1;
	while (ros::ok)
	{
		// accept
		// connectfd = accept(listenfd,(struct sockaddr *)&client,&addrlen);
		if (connectfd < 0)
		{
			printf("connect() error \n");
			return -1;
		}
			//printf("connect() success\n");
		// printf("You got a connection from client's IP is %s, port is %d\n",
		//		inet_ntoa(client.sin_addr), ntohs(client.sin_port));
        cout<<"settingmsg"<<endl;
		msgMotorTemplate motorData = msgMotorTemplate(msg_tocamera_count++, 0, jointActualAngles, 0, {0, 0, 0, 0, 0}, "Motor data for test!");
		if (msg_tocamera_count >1000)
		{
			msg_tocamera_count=0;
		}
		//display(motorData);
		//send(connectfd, "1111",4,0);

		i_count++;
		if(i_count==20)
		{
		i_count=0;		
		
        cout<<"send_msg"<<endl;
		 if(!send(connectfd, (stdMsgMotor(motorData).char_msg), 256, 0))
		 {
			cout<<"Send Error!"<<endl;
		 }
			cout<<"Send success"<<endl;
		}

//zhushi
		size_recv = recv(connectfd, buf, 256, 0);
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

			if (size_recv % 256 != 0)
			{
				cout << "Received message has a wrong length (supposed to be 256)" << endl;
				cout<<"receive_length:" <<size_recv<<endl;
			}
			
			else
			{
				for (int i = 0; i < 256; i++)
				{
					msg_char[i] = buf[i];
				}
				//if((msg_char).msg.id)
				//display(stdMsgCamera(msg_char).msg);

				//flush the receive buffer per 100 msg
				msgCameraTemplate camMsg = stdMsgCamera(msg_char).msg;
				if(camMsg.id%100==0)
				{
					display(camMsg);
					int i=0;
					while(recv(connectfd, buf, BUFF_SIZE, 0)==BUFF_SIZE)
					{
						i++;
						cout<<i<<endl;
					}
				}
			}
		}

		
//zhushi





//zhushi
        sensor_msgs::JointState joint_state;
		joint_state.header.stamp = ros::Time::now();
        joint_state.position.resize(5);
        //joint_state.position[0] = expect_q;
        //joint_state.position[1] = joint_1.get_current_angle();
        joint_state.position[1] = stdMsgCamera(msg_char).msg.jointDesiredAngles[0];
        joint_state.position[2] = stdMsgCamera(msg_char).msg.jointDesiredAngles[1];
        joint_state.position[3] = stdMsgCamera(msg_char).msg.jointDesiredAngles[2];

        joint_state.position[4] = (double)stdMsgCamera(msg_char).msg.state;

		//ROS_INFO("mainpulator joint currentangle  %lf   %lf   %lf",joint_state.position[1],joint_state.position[2],joint_state.position[3]);

		//ROS_INFO("mainpulator joint currentangle  STATE %d  ",stdMsgCamera(msg_char).msg.state);
		//ROS_INFO("mainpulator joint currentangle  STATE %lf   ",joint_state.position[4]);


        pub.publish(joint_state);
        
//zhushi




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
