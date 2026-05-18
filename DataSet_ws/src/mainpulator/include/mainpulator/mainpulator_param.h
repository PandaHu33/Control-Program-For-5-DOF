#ifndef H_MAINPULATOR_PARAM
#define H_MAINPULATOR_PARAM
#include "mainpulator/mainpulator_control.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string.h>
#include <iostream>

#include <ros/ros.h>

#include <ros/spinner.h>

#include <std_msgs/String.h> 

#include <std_msgs/Bool.h>

#include<tf/transform_broadcaster.h>

#include<nav_msgs/Odometry.h>

#include<geometry_msgs/Twist.h>

#include <std_msgs/String.h>

#include<geometry_msgs/Point.h>
#include <can_msgs/Frame.h>

#include <socketcan_interface/socketcan.h>

#include <sensor_msgs/JointState.h>

#include <robot_state_publisher/robot_state_publisher.h>

namespace param{

	/*清除错误*/
	char clear[8] = { 0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00 };
	/*设定位置模式1*/
	char setstates[8] = { 0x2F,0x60,0x60,0x00,0x04,0x00,0x00,0x00 };
	/*回读当前模式*/
	char readstate[8] = { 0x40,0x61,0x60,0x00,0x00,0x00,0x00,0x00 };
	/*读取当前位置*/
	char readPosition[8] = { 0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00 };
	/*写入目标位置 后四位为写入目标位置，先低后高*/
	char setposition[8] = { 0x23,0x7A,0x60,0x00,0x00,0x00,0x00,0x00 };
	/*写入加速度 后四位为写入目标位置，先低后高*/
	char setacceleration[8] = { 0x23,0x83,0x60,0x00,0x00,0x00,0x00,0x00 };
	/*写入减速度 后四位为写入目标位置，先低后高*/
	char setdeceleration[8] = { 0x23,0x84,0x60,0x00,0x00,0x00,0x00,0x00 };
	/*写入速度 后四位为写入目标位置，先低后高*/
	char setVelocity[8] = { 0x23,0x81,0x60,0x00,0x00,0x00,0x00,0x00 };
	/*停止电机*/
	char stop[8] = { 0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00 };
	/*伺服准备*/
	char servo[8] = { 0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00 };
	char enable[8] = { 0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00 };
	/*开始运动*/
	char start[8] = { 0x2B,0x40,0x60,0x00,0x1F,0x00,0x00,0x00 };
	/*读取速度*/
	char readVelocity[8] = { 0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00 };
	char setMaxVelocity[8] = { 0x2B,0xC5,0x60,0x00,0x0F,0x00,0x00,0x00 };
	char getA[8] = { 0x40,0x75,0x60,0x00,0x00,0x00,0x00,0x00 };
	char setA[8] = { 0x2B,0x71,0x60,0x00,0x00,0x00,0x00,0x00 };
	/*写入最大电流*/
	char setMaxI[8] = {0x2B,0x73,0x60,0x00,0x00,0x00,0x00,0x00};
	/*复位节点*/
	char reset[2] = { 0x82,0x00 };

	char buffer[8] = { 0 };
	/*关闭同步发生器*/
	char closePDO[8] = { 0x23,0x05,0x10,0x00,0x80,0x00,0x00,0x00 };
	/*设定同步时间*/
	char setTimes[8] = { 0x23,0x06,0x10,0x00,0xA0,0x0F,0x00,0x00 };
	//char setTimes[8] = { 0x23,0x06,0x10,0x00,0xD0,0x07,0x00,0x00 };
	/*************TPDO**************************************************/
	/*无效TPOD，
	索引：1800+PDO_ID-1
	子索引： 01
	数据位：180+Nede_ID～480+Nede_ID 00 80*/
	char closeTPDO[8] = { 0x23,0x00,0x18,0x01,0x80,0x00,0x00,0x80 };
	/*设定TPOD1传输类型为1
	索引：1800+PDO_ID -1
	子索引： 02
	数据位：01 00 00 00*/
	char setTPDOtype[8] = { 0x2F,0x00,0x18,0x02,0x01,0x00,0x00,0x00 };
	/*设定TPDO1传输禁止时间
	索引：1800+PDO_ID -1
	子索引： 03
	数据位：00 00 00 00 单位100us*/
	char  setPhbTime[8] = { 0x2B,0x00,0x18,0x03,0x64,0x00,0x00,0x00 };
	/*TPOD1子索引清零
	索引 1A00~1A03
	*/
	char TPDOmapclear[8] = { 0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00 };
	/*映射关系
	索引： 1A00h~1A03
	子索引： 00
	对象长度： 00
	控制字对象：  00 00 00
	*/
	char TPDOmap[8] = { 0x23,0x00,0x1A,0x00,0x00,0x00,0x00,0x00 };
	/*设定TPDO1子索引数目为
	索引：1A00 ～ 1A03
	子索引： 00
	数据位： 00 00 00 00 先底后高
	*/
	char TPDOmapnum[8] = { 0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00 };
	/*激活PDO
	索引：1800+PDO_ID -1
	子索引：01
	数据位：180+Nede_ID～480+Nede_ID 00 80
	*/
	char openTPOD[8] = { 0x23,0x00,0x18,0x01,0x80,0x01,0x00,0x00 };
	/***********************RPDO***************************************************/

	/*无效RPOD，
	索引：1400+PDO_ID-1
	子索引： 01
	数据位：200+Nede_ID～400+Nede_ID 00 80*/
	char closeRPDO[8] = { 0x23,0x00,0x14,0x01,0x80,0x00,0x00,0x80 };
	/*设定TPOD1传输类型为1
	索引：1400+PDO_ID -1
	子索引： 02
	数据位：01 00 00 00*/
	char setRPDOtype[8] = { 0x2F,0x00,0x14,0x02,0x01,0x00,0x00,0x00 };
	/*设定TPDO1传输禁止时间
	索引：1800+PDO_ID -1
	子索引： 03
	数据位：00 00 00 00 单位100us*/
	//char  setPhbTime1[8] = {0x2B,0x00,0x18,0x03,0x64,0x00,0x00,0x00};



	/*RPOD1子索引清零
	索引 1600~1603
	*/
	char RPDOmapclear[8] = { 0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00 };
	/*映射关系
	索引： 1600h~1603
	子索引： 00
	对象长度： 00
	控制字对象：  00 00 00
	*/
	char RPDOmap[8] = { 0x23,0x00,0x16,0x00,0x00,0x00,0x00,0x00 };
	/*设定TPDO1子索引数目为
	索引：1600 ～ 1603
	子索引： 00
	数据位： 00 00 00 00 先底后高
	*/
	char RPDOmapnum[8] = { 0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00 };
	/*激活PDO
	索引：1400+PDO_ID -1
	子索引：01
	数据位：200+Nede_ID～500+Nede_ID 00 00
	*/
	char openRPOD[8] = { 0x23,0x00,0x14,0x01,0x80,0x01,0x00,0x00 };
	/**************************************************************************/


	char openPOD[8] = { 0x23,0x05,0x10,0x00,0x80,0x00,0x00,0x40 };

	#define CAN_EFF_FLAG 0x80000000U //扩展帧的标识
	#define CAN_RTR_FLAG 0x40000000U //远程帧的标识
	#define CAN_ERR_FLAG 0x20000000U //错误帧的标识用于检查错误

	typedef struct CAN_SDO
	{

		char setstates[8];//设定状态   
		char readstate[8];//回读状态
		char readPosition[8];//读取当前位置
		char setposition[8];//写入目标位置 后四位为写入目标位置，先低后高
		char setacceleration[8];//写入加速度 后四位为写入目标位置，先低后高
		char setdeceleration[8];//写入减速度 后四位为写入目标位置，先低后高
		char setVelocity[8];//写入速度 后四位为写入目标位置，先低后高
		char stop[8];//停止电机
		char servo[8];//伺服准备
		char enable[8];//
		char start[8];//
		char readVelocity[8];//

		char setMaxVelocity[8];

		char setTimes[8]; //设定同步时间
		char closePDO[8]; //关闭同步发生器
		char openPOD[8];
		char setA[8];
		char getA[8];
	}SDO;//SDO模式传输模式

	typedef struct CAN_TPOD
	{
		/*无效TPOD，
		索引：1800+PDO_ID-1
		子索引： 01
		数据位：180+Nede_ID～480+Nede_ID 00 80*/
		char close[8];
		/*设定TPOD1传输类型为1
		索引：1800+PDO_ID -1
		子索引： 02
		数据位：01 00 00 00*/
		char Type[8];
		/*设定TPDO1传输禁止时间
		索引：1800+PDO_ID -1
		子索引： 03
		数据位：00 00 00 00 单位100us*/
		char  PhbTime[8];
		/*TPOD1子索引清零
		索引 1A00~1A03
		*/
		char MapClear[8];
		/*映射关系
		索引： 1A00h~1A03
		子索引： 00
		对象长度： 00
		控制字对象：  00 00 00
		*/
		char Map[8];
		/*设定TPDO1子索引数目为
		索引：1A00 ～ 1A03
		子索引： 00
		数据位： 00 00 00 00 先底后高
		*/
		char Mapnum[8];
		/*激活PDO
		索引：1800+PDO_ID -1
		子索引：01
		数据位：180+Nede_ID～480+Nede_ID 00 80
		*/
		char open[8];

	}TPDO;//设置TPDO

	typedef struct CAN_RPDO
	{
		/*无效RPOD，
		索引：1400+PDO_ID-1
		子索引： 01
		数据位：200+Nede_ID～400+Nede_ID 00 80*/
		char close[8];
		/*设定TPOD1传输类型为1
		索引：1400+PDO_ID -1
		子索引： 02
		数据位：01 00 00 00*/
		char Type[8];
		/*RPOD1子索引清零
		索引 1600~1603 */
		char Mapclear[8];
		/*映射关系
		索引： 1600h~1603
		子索引： 00
		对象长度： 00
		控制字对象：  00 00 00 */
		char Map[8];
		/*设定TPDO1子索引数目为
		索引：1600 ～ 1603
		子索引： 00
		数据位： 00 00 00 00 先底后高*/
		char Mapnum[8];
		/*激活PDO
		索引：1400+PDO_ID -1
		子索引：01
		数据位：200+Nede_ID～500+Nede_ID 00 00
		*/
		char open[8];
	}RPDO;//设置RRPD


	int SocketCANInit();

	void SocketWrite(const struct can_frame frames,const int sock_fd);

	void SocketRead(struct can_frame&  frames,const int sock_fd);

	void PositionInit(control::mainpulator& joint,const int sock_fd);

	void Enable(const int sock_fd,int length,control::mainpulator& joint);

	void MomentInit(control::mainpulator& joint,const int sock_fd);

	//void MomentEnable(const int sock_fd,int length,control::mainpulator& joint);
	


}













#endif