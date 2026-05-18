/*
 * File name: mavlink_udp.h
 * Purpose: On POSIX API, simulate a vehicle to connect to QGC via MAVLink over UDP, and send/receive the messages.(Tested on Linux)
 * Creating Date: 2019.11.27
 * Author/Charge: Panda Wang (lucky00122@gmail.com)
 * Note: N/A
 */
//#pragma once

#ifndef MAVLINK_UDP_H
#define MAVLINK_UDP_H

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */

#include "mavlink/ardupilotmega/mavlink.h"
#include "mavlink/ardupilotmega/ardupilotmega.h"
#include "mavlink/common/common.h"
// #define CLIENT_IP		"192.168.2.128"
#define CLIENT_IP		"192.168.1.11" //xian kong de ip
#define CLIENT_PORT		14550	// QGC UDP port
#define SERVER_PORT		14551
#define SYSTEM_ID		1
#define COMPONENT_ID	MAV_COMP_ID_AUTOPILOT1
#define BUFFER_LENGTH	2041 // minimum buffer size that can be used with qnx (I don't know why)



// Parameter Name Definitions
#define PARAM_NAME_ALTITUDE	"altitude"
#define AP_MAV_PROTOCOL_CAPABILITY MAV_PROTOCOL_CAPABILITY_MAVLINK2|\
									MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
// GLOBAL_POSITION_INT data
#define VEHICLE_GPS_LATITUDE	(float)30.286747
#define VEHICLE_GPS_LONGITUDE	(float)119.972913
#define GET_GPS_SCALED_VALUE(degree)	(int32_t)((float)degree*10000000)
#define VEHICLE_ALTITUDE_MSL	500000	// mm
#define VEHICLE_ALTITUDE_GND	400000	// mm


 /******************COLOR PRINTING DEFINITIONS *******************/
#define ESC_COLOR_NORMAL   "\033[m"
#define ESC_COLOR_BLACK    "\033[30m"
#define ESC_COLOR_RED      "\033[31m"
#define ESC_COLOR_GREEN    "\033[32m"
#define ESC_COLOR_YELLOW   "\033[33m"
#define ESC_COLOR_BLUE     "\033[34m"
#define ESC_COLOR_MAGENTA  "\033[35m"
#define ESC_COLOR_CYAN     "\033[36m"
#define ESC_COLOR_WHITE    "\033[37m"


/******************COLOR PRINTING DEFINITIONS *******************/
#define NORMAL( x )  ESC_COLOR_NORMAL  x
#define BLACK( x )   ESC_COLOR_BLACK   x ESC_COLOR_NORMAL
#define RED( x )     ESC_COLOR_RED     x ESC_COLOR_NORMAL
#define GREEN( x )   ESC_COLOR_GREEN   x ESC_COLOR_NORMAL
#define YELLOW( x )  ESC_COLOR_YELLOW  x ESC_COLOR_NORMAL
#define BLUE( x )    ESC_COLOR_BLUE    x ESC_COLOR_NORMAL
#define MAGENTA( x ) ESC_COLOR_MAGENTA x ESC_COLOR_NORMAL
#define CYAN( x )    ESC_COLOR_CYAN    x ESC_COLOR_NORMAL
#define WHITE( x )   ESC_COLOR_WHITE   x ESC_COLOR_NORMAL

#define printf_color(color, str, args...)   printf( color str ESC_COLOR_NORMAL "\r", ##args )
#define fprintf_color(color, str, args...)  fprintf( stderr, color str ESC_COLOR_NORMAL "\r", ##args )

#define printf_black(str, args...)     printf_color(  ESC_COLOR_BLACK,   str, ##args )
#define fprintf_black(str, args...)    fprintf_color( ESC_COLOR_BLACK,   str, ##args )
#define printf_red(str, args...)       printf_color(  ESC_COLOR_RED,     str, ##args )
#define fprintf_red(str, args...)      fprintf_color( ESC_COLOR_RED,     str, ##args )
#define printf_green(str, args...)     printf_color(  ESC_COLOR_GREEN,   str, ##args )
#define fprintf_green(str, args...)    fprintf_color( ESC_COLOR_GREEN,   str, ##args )
#define printf_yellow(str, args...)    printf_color(  ESC_COLOR_YELLOW,  str, ##args )
#define fprintf_yellow(str, args...)   fprintf_color( ESC_COLOR_YELLOW,  str, ##args )
#define printf_blue(str, args...)      printf_color(  ESC_COLOR_BLUE,    str, ##args )
#define fprintf_blue(str, args...)     fprintf_color( ESC_COLOR_BLUE,    str, ##args )
#define fprintf_magenta(str, args...)  fprintf_color( ESC_COLOR_MAGENTA, str, ##args )
#define printf_magenta(str, args...)   printf_color(  ESC_COLOR_MAGENTA, str, ##args )
#define fprintf_cyan(str, args...)     fprintf_color( ESC_COLOR_CYAN,    str, ##args )
#define printf_cyan(str, args...)      printf_color(  ESC_COLOR_CYAN,    str, ##args )
#define fprintf_white(str, args...)    fprintf_color( ESC_COLOR_WHITE,   str, ##args )
#define printf_white(str, args...)     printf_color(  ESC_COLOR_WHITE,   str, ##args )

#define PRINT_FUNC	printf(GREEN("%s\n"), __FUNCTION__);

typedef enum _AP_PARAM_INDEX{
	AP_PARAM_INDEX_ALTITUDE,
	AP_PARAM_INDEX_TOTAL,
}AP_PARAM_INDEX;
	
uint64_t microsSinceEpoch();
void AP_Decode_Message(mavlink_message_t* msg);
void AP_Decode_Command_Long(mavlink_command_long_t* cmd);
void AP_Send_Message_Param_Value(uint16_t param_index);
void AP_Send_Message_Mission_count(uint8_t mission_type);

#endif	/* MAVLINK_UDP_H */