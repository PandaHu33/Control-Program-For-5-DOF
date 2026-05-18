#define _WINSOCK_DEPRECATED_NO_WARNINGS 	
#define _CRT_SECURE_NO_WARNINGS

#define TCP_MSG_SIZE 256

#include <stdio.h>
#include <stdlib.h>
#include<iostream>
#include<string>
#include<cstring>
#include<vector>

#pragma comment(lib, "ws2_32.lib") 			//ÏÔÊŸŒÓÔØ ws2_32.dll	ws2_32.dllŸÍÊÇ×îÐÂsocket°æ±ŸÀ²

const int len_double = sizeof(double);
const int len_int = sizeof(int);
typedef union doubletochar_8
{
	double num_double;
	char num_char[len_double];
}doubleMsg;
typedef union inttochar_4
{
	double int_double;
	char int_char[len_int];
}intMsg;

//Ïà»ú¶Ë·¢ËÍµÄÐÅÏ¢žñÊœ
struct msgCameraTemplate
{
	int length, id, state;//12byte
	int reservedTag = 0;//4byte��reservedTagĬ��Ϊ0
	double jointDesiredAngles[4];//len4, 32byte
	double clamperDesiredSpace;//len1, 8byte
	double targetRelativePose[6];//len6, 48byte
	double targetGlobalPosition[3];//len3, 24byte
	double objectivePointGlobalPosition[3];//len3, 24byte
	double reservedData[6] = {0};//len6, 48byte��Ĭ��Ϊ0
	char noteStr[56] = "";//len56, 56byte��Ĭ��Ϊ'\0'
	msgCameraTemplate(int id, int state, std::vector<double> jointDesiredAngles, double clamperDesiredSpace, \
		std::vector<double> targetRelativePose, std::vector<double> targetGlobalPosition, \
		std::vector<double> objectivePointGlobalPosition, std::string noteStr)
	{
		this->length = 256;
		this->id = id;
		this->state = state;
		//this->reservedTag = 0;//������ǩλ��ʼΪ0
		if (jointDesiredAngles.size() != 4) {
			
			throw - 1;
		}
		for (int i = 0; i < 4; i++) this->jointDesiredAngles[i] = jointDesiredAngles[i];
		this->clamperDesiredSpace = clamperDesiredSpace;
		if (targetRelativePose.size() != 6) {
			
			throw - 1;
		}
		for (int i = 0; i < 6; i++) this->targetRelativePose[i] = targetRelativePose[i];
		if (targetGlobalPosition.size() != 3) {
			
			throw - 1;
		}
		for (int i = 0; i < 3; i++) this->targetGlobalPosition[i] = targetGlobalPosition[i];
		if (objectivePointGlobalPosition.size() != 3) {
			
			throw - 1;
		}
		for (int i = 0; i < 3; i++) this->objectivePointGlobalPosition[i] = objectivePointGlobalPosition[i];
		//for (int i = 0; i < 6; i++) this->reservedData[i] = 0.0;//��������λ��ʼΪ0
		if (noteStr.length() > 56) {
			
			throw - 1;
		}
		//this->noteStr = "";//��ע�ַ�����ʼΪ'\0'
		for (int i = 0; i < noteStr.length(); i++) {
			this->noteStr[i] = noteStr[i];
		}
	}
	msgCameraTemplate()
	{
		*this = msgCameraTemplate(0, 0, { 0,0,0,0 }, 0, { 0, 0, 0, 0, 0, 0 }, { 0,0,0 }, { 0,0,0 }, "");
	}
};
typedef union msgCameraTemplateToChar256
{
	char char_msg[256];
	msgCameraTemplate msg;
	msgCameraTemplateToChar256(msgCameraTemplate msg) {
		this->msg = msg;
	}
	msgCameraTemplateToChar256(char char_msg[256]) {
		for (int i = 0; i < 256; i++) this->char_msg[i] = char_msg[i];
	}
	msgCameraTemplateToChar256() {
		for (int i = 0; i < 256; i++) this->char_msg[i] = '\0';
		this->msg = msgCameraTemplate();
	}
}stdMsgCamera;
void display(msgCameraTemplate msg);
//void display(msgCameraTemplate msg);

//µç»ú¶Ë·¢ËÍµÄÐÅÏ¢žñÊœ
struct msgMotorTemplate
{
	int length, id, state;//12byte
	int reservedTag = 0;//4byte��reservedTagĬ��Ϊ0
	double jointActualAngles[4];//len4, 32byte
	double clamperActualSpace;//len1, 8byte
	double motorVelocity[5];//len5, 40byte
	double reservedData[13] = { 0 };//len13, 104byte��Ĭ��Ϊ0
	char noteStr[56] = "";//len56, 56byte��Ĭ��Ϊ'\0'
	msgMotorTemplate(int id, int state, std::vector<double> jointActualAngles, double clamperActualSpace, \
		std::vector<double> motorVelocity, std::string noteStr)
	{
		this->length = 256;
		this->id = id;
		this->state = state;
		//this->reservedTag = 0;//������ǩλ��ʼΪ0
		if (jointActualAngles.size() != 4) {
			
			throw - 1;
		}
		for (int i = 0; i < 4; i++) this->jointActualAngles[i] = jointActualAngles[i];
		this->clamperActualSpace = clamperActualSpace;
		if (motorVelocity.size() != 5) {
			
			throw - 1;
		}
		//for (int i = 0; i < 13; i++) this->reservedData[i] = 0.0;//��������λ��ʼΪ0
		if (noteStr.length() > 56) {
			
			throw - 1;
		}
		//this->noteStr = "";//��ע�ַ�����ʼΪ'\0'
		for (int i = 0; i < noteStr.length(); i++) {
			this->noteStr[i] = noteStr[i];
		}
	}
	msgMotorTemplate()
	{
		*this = msgMotorTemplate(0, 0, { 0,0,0,0 }, 0, { 0, 0, 0, 0, 0}, "");
	}
};
typedef union msgMotorTemplateToChar256
{
	char char_msg[256];
	msgMotorTemplate msg;
	msgMotorTemplateToChar256(msgMotorTemplate msg) {
		this->msg = msg;
	}
	msgMotorTemplateToChar256(char char_msg[256]) {
		for (int i = 0; i < 256; i++) this->char_msg[i] = char_msg[i];
	}
	msgMotorTemplateToChar256() {
		for (int i = 0; i < 256; i++) this->char_msg[i] = '\0';
		this->msg = msgMotorTemplate();
	}
}stdMsgMotor;
void display(msgMotorTemplate msg);

//void display(msgMotorTemplate msg);