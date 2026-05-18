#pragma once
// MESSAGE ROV_ACOEF PACKING

#define MAVLINK_MSG_ID_ROV_ACOEF 240


typedef struct __mavlink_rov_acoef_t {
 uint32_t CommuFreq; /*<  mainboard communication frequency.*/
 float MaxSAng; /*<  Max servo angle.*/
 float MinSAng; /*<  Min servo angle.*/
 float MaxTSpd; /*<  Maximum thruster speed.*/
 float DepthPID; /*<  Depth PID control coefficients.*/
 float YawPID; /*<  Yaw PID control coefficients.*/
 float PosCtrl; /*<  Position control coefficients.*/
 float AvoiCtrl; /*<  Obstacle avoidance coefficients.*/
 float alternate_rov_acoef_float_1; /*<  alternate_rov_acoef_float_1.*/
 float alternate_rov_acoef_float_2; /*<  alternate_rov_acoef_float_2.*/
 float alternate_rov_acoef_float_3; /*<  alternate_rov_acoef_float_3.*/
 float alternate_rov_acoef_float_4; /*<  alternate_rov_acoef_float_4.*/
 float alternate_rov_acoef_float_5; /*<  alternate_rov_acoef_float_5.*/
 uint32_t alternate_rov_acoef_int_4; /*<  alternate_rov_acoef_int_4.*/
 uint32_t alternate_rov_acoef_int_5; /*<  alternate_rov_acoef_int_5.*/
 uint8_t MovCtrl; /*<  ON/OFFs-motion control thread.*/
 uint8_t ServoCtrl; /*<  ON/OFFs-servo control thread.*/
 char MCtrlIP[40]; /*<  IP addr. of motion controller.*/
 uint8_t alternate_rov_acoef_int_1; /*<  alternate_rov_acoef_int_1.*/
 uint8_t alternate_rov_acoef_int_2; /*<  alternate_rov_acoef_int_2.*/
 uint8_t alternate_rov_acoef_int_3; /*<  alternate_rov_acoef_int_3.*/
} mavlink_rov_acoef_t;

#define MAVLINK_MSG_ID_ROV_ACOEF_LEN 105
#define MAVLINK_MSG_ID_ROV_ACOEF_MIN_LEN 105
#define MAVLINK_MSG_ID_240_LEN 105
#define MAVLINK_MSG_ID_240_MIN_LEN 105

#define MAVLINK_MSG_ID_ROV_ACOEF_CRC 241
#define MAVLINK_MSG_ID_240_CRC 241

#define MAVLINK_MSG_ROV_ACOEF_FIELD_MCTRLIP_LEN 40

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROV_ACOEF { \
    240, \
    "ROV_ACOEF", \
    21, \
    {  { "MovCtrl", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_rov_acoef_t, MovCtrl) }, \
         { "ServoCtrl", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_rov_acoef_t, ServoCtrl) }, \
         { "CommuFreq", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_rov_acoef_t, CommuFreq) }, \
         { "MaxSAng", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rov_acoef_t, MaxSAng) }, \
         { "MinSAng", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rov_acoef_t, MinSAng) }, \
         { "MCtrlIP", NULL, MAVLINK_TYPE_CHAR, 40, 62, offsetof(mavlink_rov_acoef_t, MCtrlIP) }, \
         { "MaxTSpd", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rov_acoef_t, MaxTSpd) }, \
         { "DepthPID", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rov_acoef_t, DepthPID) }, \
         { "YawPID", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rov_acoef_t, YawPID) }, \
         { "PosCtrl", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rov_acoef_t, PosCtrl) }, \
         { "AvoiCtrl", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rov_acoef_t, AvoiCtrl) }, \
         { "alternate_rov_acoef_float_1", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_1) }, \
         { "alternate_rov_acoef_float_2", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_2) }, \
         { "alternate_rov_acoef_float_3", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_3) }, \
         { "alternate_rov_acoef_float_4", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_4) }, \
         { "alternate_rov_acoef_float_5", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_5) }, \
         { "alternate_rov_acoef_int_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 102, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_1) }, \
         { "alternate_rov_acoef_int_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 103, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_2) }, \
         { "alternate_rov_acoef_int_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 104, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_3) }, \
         { "alternate_rov_acoef_int_4", NULL, MAVLINK_TYPE_UINT32_T, 0, 52, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_4) }, \
         { "alternate_rov_acoef_int_5", NULL, MAVLINK_TYPE_UINT32_T, 0, 56, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_5) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROV_ACOEF { \
    "ROV_ACOEF", \
    21, \
    {  { "MovCtrl", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_rov_acoef_t, MovCtrl) }, \
         { "ServoCtrl", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_rov_acoef_t, ServoCtrl) }, \
         { "CommuFreq", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_rov_acoef_t, CommuFreq) }, \
         { "MaxSAng", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rov_acoef_t, MaxSAng) }, \
         { "MinSAng", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rov_acoef_t, MinSAng) }, \
         { "MCtrlIP", NULL, MAVLINK_TYPE_CHAR, 40, 62, offsetof(mavlink_rov_acoef_t, MCtrlIP) }, \
         { "MaxTSpd", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rov_acoef_t, MaxTSpd) }, \
         { "DepthPID", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rov_acoef_t, DepthPID) }, \
         { "YawPID", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rov_acoef_t, YawPID) }, \
         { "PosCtrl", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rov_acoef_t, PosCtrl) }, \
         { "AvoiCtrl", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rov_acoef_t, AvoiCtrl) }, \
         { "alternate_rov_acoef_float_1", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_1) }, \
         { "alternate_rov_acoef_float_2", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_2) }, \
         { "alternate_rov_acoef_float_3", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_3) }, \
         { "alternate_rov_acoef_float_4", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_4) }, \
         { "alternate_rov_acoef_float_5", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_float_5) }, \
         { "alternate_rov_acoef_int_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 102, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_1) }, \
         { "alternate_rov_acoef_int_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 103, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_2) }, \
         { "alternate_rov_acoef_int_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 104, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_3) }, \
         { "alternate_rov_acoef_int_4", NULL, MAVLINK_TYPE_UINT32_T, 0, 52, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_4) }, \
         { "alternate_rov_acoef_int_5", NULL, MAVLINK_TYPE_UINT32_T, 0, 56, offsetof(mavlink_rov_acoef_t, alternate_rov_acoef_int_5) }, \
         } \
}
#endif

/**
 * @brief Pack a rov_acoef message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param MovCtrl  ON/OFFs-motion control thread.
 * @param ServoCtrl  ON/OFFs-servo control thread.
 * @param CommuFreq  mainboard communication frequency.
 * @param MaxSAng  Max servo angle.
 * @param MinSAng  Min servo angle.
 * @param MCtrlIP  IP addr. of motion controller.
 * @param MaxTSpd  Maximum thruster speed.
 * @param DepthPID  Depth PID control coefficients.
 * @param YawPID  Yaw PID control coefficients.
 * @param PosCtrl  Position control coefficients.
 * @param AvoiCtrl  Obstacle avoidance coefficients.
 * @param alternate_rov_acoef_float_1  alternate_rov_acoef_float_1.
 * @param alternate_rov_acoef_float_2  alternate_rov_acoef_float_2.
 * @param alternate_rov_acoef_float_3  alternate_rov_acoef_float_3.
 * @param alternate_rov_acoef_float_4  alternate_rov_acoef_float_4.
 * @param alternate_rov_acoef_float_5  alternate_rov_acoef_float_5.
 * @param alternate_rov_acoef_int_1  alternate_rov_acoef_int_1.
 * @param alternate_rov_acoef_int_2  alternate_rov_acoef_int_2.
 * @param alternate_rov_acoef_int_3  alternate_rov_acoef_int_3.
 * @param alternate_rov_acoef_int_4  alternate_rov_acoef_int_4.
 * @param alternate_rov_acoef_int_5  alternate_rov_acoef_int_5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rov_acoef_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t MovCtrl, uint8_t ServoCtrl, uint32_t CommuFreq, float MaxSAng, float MinSAng, const char *MCtrlIP, float MaxTSpd, float DepthPID, float YawPID, float PosCtrl, float AvoiCtrl, float alternate_rov_acoef_float_1, float alternate_rov_acoef_float_2, float alternate_rov_acoef_float_3, float alternate_rov_acoef_float_4, float alternate_rov_acoef_float_5, uint8_t alternate_rov_acoef_int_1, uint8_t alternate_rov_acoef_int_2, uint8_t alternate_rov_acoef_int_3, uint32_t alternate_rov_acoef_int_4, uint32_t alternate_rov_acoef_int_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROV_ACOEF_LEN];
    _mav_put_uint32_t(buf, 0, CommuFreq);
    _mav_put_float(buf, 4, MaxSAng);
    _mav_put_float(buf, 8, MinSAng);
    _mav_put_float(buf, 12, MaxTSpd);
    _mav_put_float(buf, 16, DepthPID);
    _mav_put_float(buf, 20, YawPID);
    _mav_put_float(buf, 24, PosCtrl);
    _mav_put_float(buf, 28, AvoiCtrl);
    _mav_put_float(buf, 32, alternate_rov_acoef_float_1);
    _mav_put_float(buf, 36, alternate_rov_acoef_float_2);
    _mav_put_float(buf, 40, alternate_rov_acoef_float_3);
    _mav_put_float(buf, 44, alternate_rov_acoef_float_4);
    _mav_put_float(buf, 48, alternate_rov_acoef_float_5);
    _mav_put_uint32_t(buf, 52, alternate_rov_acoef_int_4);
    _mav_put_uint32_t(buf, 56, alternate_rov_acoef_int_5);
    _mav_put_uint8_t(buf, 60, MovCtrl);
    _mav_put_uint8_t(buf, 61, ServoCtrl);
    _mav_put_uint8_t(buf, 102, alternate_rov_acoef_int_1);
    _mav_put_uint8_t(buf, 103, alternate_rov_acoef_int_2);
    _mav_put_uint8_t(buf, 104, alternate_rov_acoef_int_3);
    _mav_put_char_array(buf, 62, MCtrlIP, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROV_ACOEF_LEN);
#else
    mavlink_rov_acoef_t packet;
    packet.CommuFreq = CommuFreq;
    packet.MaxSAng = MaxSAng;
    packet.MinSAng = MinSAng;
    packet.MaxTSpd = MaxTSpd;
    packet.DepthPID = DepthPID;
    packet.YawPID = YawPID;
    packet.PosCtrl = PosCtrl;
    packet.AvoiCtrl = AvoiCtrl;
    packet.alternate_rov_acoef_float_1 = alternate_rov_acoef_float_1;
    packet.alternate_rov_acoef_float_2 = alternate_rov_acoef_float_2;
    packet.alternate_rov_acoef_float_3 = alternate_rov_acoef_float_3;
    packet.alternate_rov_acoef_float_4 = alternate_rov_acoef_float_4;
    packet.alternate_rov_acoef_float_5 = alternate_rov_acoef_float_5;
    packet.alternate_rov_acoef_int_4 = alternate_rov_acoef_int_4;
    packet.alternate_rov_acoef_int_5 = alternate_rov_acoef_int_5;
    packet.MovCtrl = MovCtrl;
    packet.ServoCtrl = ServoCtrl;
    packet.alternate_rov_acoef_int_1 = alternate_rov_acoef_int_1;
    packet.alternate_rov_acoef_int_2 = alternate_rov_acoef_int_2;
    packet.alternate_rov_acoef_int_3 = alternate_rov_acoef_int_3;
    mav_array_memcpy(packet.MCtrlIP, MCtrlIP, sizeof(char)*40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROV_ACOEF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROV_ACOEF;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROV_ACOEF_MIN_LEN, MAVLINK_MSG_ID_ROV_ACOEF_LEN, MAVLINK_MSG_ID_ROV_ACOEF_CRC);
}

/**
 * @brief Pack a rov_acoef message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param MovCtrl  ON/OFFs-motion control thread.
 * @param ServoCtrl  ON/OFFs-servo control thread.
 * @param CommuFreq  mainboard communication frequency.
 * @param MaxSAng  Max servo angle.
 * @param MinSAng  Min servo angle.
 * @param MCtrlIP  IP addr. of motion controller.
 * @param MaxTSpd  Maximum thruster speed.
 * @param DepthPID  Depth PID control coefficients.
 * @param YawPID  Yaw PID control coefficients.
 * @param PosCtrl  Position control coefficients.
 * @param AvoiCtrl  Obstacle avoidance coefficients.
 * @param alternate_rov_acoef_float_1  alternate_rov_acoef_float_1.
 * @param alternate_rov_acoef_float_2  alternate_rov_acoef_float_2.
 * @param alternate_rov_acoef_float_3  alternate_rov_acoef_float_3.
 * @param alternate_rov_acoef_float_4  alternate_rov_acoef_float_4.
 * @param alternate_rov_acoef_float_5  alternate_rov_acoef_float_5.
 * @param alternate_rov_acoef_int_1  alternate_rov_acoef_int_1.
 * @param alternate_rov_acoef_int_2  alternate_rov_acoef_int_2.
 * @param alternate_rov_acoef_int_3  alternate_rov_acoef_int_3.
 * @param alternate_rov_acoef_int_4  alternate_rov_acoef_int_4.
 * @param alternate_rov_acoef_int_5  alternate_rov_acoef_int_5.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rov_acoef_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t MovCtrl,uint8_t ServoCtrl,uint32_t CommuFreq,float MaxSAng,float MinSAng,const char *MCtrlIP,float MaxTSpd,float DepthPID,float YawPID,float PosCtrl,float AvoiCtrl,float alternate_rov_acoef_float_1,float alternate_rov_acoef_float_2,float alternate_rov_acoef_float_3,float alternate_rov_acoef_float_4,float alternate_rov_acoef_float_5,uint8_t alternate_rov_acoef_int_1,uint8_t alternate_rov_acoef_int_2,uint8_t alternate_rov_acoef_int_3,uint32_t alternate_rov_acoef_int_4,uint32_t alternate_rov_acoef_int_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROV_ACOEF_LEN];
    _mav_put_uint32_t(buf, 0, CommuFreq);
    _mav_put_float(buf, 4, MaxSAng);
    _mav_put_float(buf, 8, MinSAng);
    _mav_put_float(buf, 12, MaxTSpd);
    _mav_put_float(buf, 16, DepthPID);
    _mav_put_float(buf, 20, YawPID);
    _mav_put_float(buf, 24, PosCtrl);
    _mav_put_float(buf, 28, AvoiCtrl);
    _mav_put_float(buf, 32, alternate_rov_acoef_float_1);
    _mav_put_float(buf, 36, alternate_rov_acoef_float_2);
    _mav_put_float(buf, 40, alternate_rov_acoef_float_3);
    _mav_put_float(buf, 44, alternate_rov_acoef_float_4);
    _mav_put_float(buf, 48, alternate_rov_acoef_float_5);
    _mav_put_uint32_t(buf, 52, alternate_rov_acoef_int_4);
    _mav_put_uint32_t(buf, 56, alternate_rov_acoef_int_5);
    _mav_put_uint8_t(buf, 60, MovCtrl);
    _mav_put_uint8_t(buf, 61, ServoCtrl);
    _mav_put_uint8_t(buf, 102, alternate_rov_acoef_int_1);
    _mav_put_uint8_t(buf, 103, alternate_rov_acoef_int_2);
    _mav_put_uint8_t(buf, 104, alternate_rov_acoef_int_3);
    _mav_put_char_array(buf, 62, MCtrlIP, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROV_ACOEF_LEN);
#else
    mavlink_rov_acoef_t packet;
    packet.CommuFreq = CommuFreq;
    packet.MaxSAng = MaxSAng;
    packet.MinSAng = MinSAng;
    packet.MaxTSpd = MaxTSpd;
    packet.DepthPID = DepthPID;
    packet.YawPID = YawPID;
    packet.PosCtrl = PosCtrl;
    packet.AvoiCtrl = AvoiCtrl;
    packet.alternate_rov_acoef_float_1 = alternate_rov_acoef_float_1;
    packet.alternate_rov_acoef_float_2 = alternate_rov_acoef_float_2;
    packet.alternate_rov_acoef_float_3 = alternate_rov_acoef_float_3;
    packet.alternate_rov_acoef_float_4 = alternate_rov_acoef_float_4;
    packet.alternate_rov_acoef_float_5 = alternate_rov_acoef_float_5;
    packet.alternate_rov_acoef_int_4 = alternate_rov_acoef_int_4;
    packet.alternate_rov_acoef_int_5 = alternate_rov_acoef_int_5;
    packet.MovCtrl = MovCtrl;
    packet.ServoCtrl = ServoCtrl;
    packet.alternate_rov_acoef_int_1 = alternate_rov_acoef_int_1;
    packet.alternate_rov_acoef_int_2 = alternate_rov_acoef_int_2;
    packet.alternate_rov_acoef_int_3 = alternate_rov_acoef_int_3;
    mav_array_memcpy(packet.MCtrlIP, MCtrlIP, sizeof(char)*40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROV_ACOEF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROV_ACOEF;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROV_ACOEF_MIN_LEN, MAVLINK_MSG_ID_ROV_ACOEF_LEN, MAVLINK_MSG_ID_ROV_ACOEF_CRC);
}

/**
 * @brief Encode a rov_acoef struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rov_acoef C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rov_acoef_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rov_acoef_t* rov_acoef)
{
    return mavlink_msg_rov_acoef_pack(system_id, component_id, msg, rov_acoef->MovCtrl, rov_acoef->ServoCtrl, rov_acoef->CommuFreq, rov_acoef->MaxSAng, rov_acoef->MinSAng, rov_acoef->MCtrlIP, rov_acoef->MaxTSpd, rov_acoef->DepthPID, rov_acoef->YawPID, rov_acoef->PosCtrl, rov_acoef->AvoiCtrl, rov_acoef->alternate_rov_acoef_float_1, rov_acoef->alternate_rov_acoef_float_2, rov_acoef->alternate_rov_acoef_float_3, rov_acoef->alternate_rov_acoef_float_4, rov_acoef->alternate_rov_acoef_float_5, rov_acoef->alternate_rov_acoef_int_1, rov_acoef->alternate_rov_acoef_int_2, rov_acoef->alternate_rov_acoef_int_3, rov_acoef->alternate_rov_acoef_int_4, rov_acoef->alternate_rov_acoef_int_5);
}

/**
 * @brief Encode a rov_acoef struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rov_acoef C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rov_acoef_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rov_acoef_t* rov_acoef)
{
    return mavlink_msg_rov_acoef_pack_chan(system_id, component_id, chan, msg, rov_acoef->MovCtrl, rov_acoef->ServoCtrl, rov_acoef->CommuFreq, rov_acoef->MaxSAng, rov_acoef->MinSAng, rov_acoef->MCtrlIP, rov_acoef->MaxTSpd, rov_acoef->DepthPID, rov_acoef->YawPID, rov_acoef->PosCtrl, rov_acoef->AvoiCtrl, rov_acoef->alternate_rov_acoef_float_1, rov_acoef->alternate_rov_acoef_float_2, rov_acoef->alternate_rov_acoef_float_3, rov_acoef->alternate_rov_acoef_float_4, rov_acoef->alternate_rov_acoef_float_5, rov_acoef->alternate_rov_acoef_int_1, rov_acoef->alternate_rov_acoef_int_2, rov_acoef->alternate_rov_acoef_int_3, rov_acoef->alternate_rov_acoef_int_4, rov_acoef->alternate_rov_acoef_int_5);
}

/**
 * @brief Send a rov_acoef message
 * @param chan MAVLink channel to send the message
 *
 * @param MovCtrl  ON/OFFs-motion control thread.
 * @param ServoCtrl  ON/OFFs-servo control thread.
 * @param CommuFreq  mainboard communication frequency.
 * @param MaxSAng  Max servo angle.
 * @param MinSAng  Min servo angle.
 * @param MCtrlIP  IP addr. of motion controller.
 * @param MaxTSpd  Maximum thruster speed.
 * @param DepthPID  Depth PID control coefficients.
 * @param YawPID  Yaw PID control coefficients.
 * @param PosCtrl  Position control coefficients.
 * @param AvoiCtrl  Obstacle avoidance coefficients.
 * @param alternate_rov_acoef_float_1  alternate_rov_acoef_float_1.
 * @param alternate_rov_acoef_float_2  alternate_rov_acoef_float_2.
 * @param alternate_rov_acoef_float_3  alternate_rov_acoef_float_3.
 * @param alternate_rov_acoef_float_4  alternate_rov_acoef_float_4.
 * @param alternate_rov_acoef_float_5  alternate_rov_acoef_float_5.
 * @param alternate_rov_acoef_int_1  alternate_rov_acoef_int_1.
 * @param alternate_rov_acoef_int_2  alternate_rov_acoef_int_2.
 * @param alternate_rov_acoef_int_3  alternate_rov_acoef_int_3.
 * @param alternate_rov_acoef_int_4  alternate_rov_acoef_int_4.
 * @param alternate_rov_acoef_int_5  alternate_rov_acoef_int_5.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rov_acoef_send(mavlink_channel_t chan, uint8_t MovCtrl, uint8_t ServoCtrl, uint32_t CommuFreq, float MaxSAng, float MinSAng, const char *MCtrlIP, float MaxTSpd, float DepthPID, float YawPID, float PosCtrl, float AvoiCtrl, float alternate_rov_acoef_float_1, float alternate_rov_acoef_float_2, float alternate_rov_acoef_float_3, float alternate_rov_acoef_float_4, float alternate_rov_acoef_float_5, uint8_t alternate_rov_acoef_int_1, uint8_t alternate_rov_acoef_int_2, uint8_t alternate_rov_acoef_int_3, uint32_t alternate_rov_acoef_int_4, uint32_t alternate_rov_acoef_int_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROV_ACOEF_LEN];
    _mav_put_uint32_t(buf, 0, CommuFreq);
    _mav_put_float(buf, 4, MaxSAng);
    _mav_put_float(buf, 8, MinSAng);
    _mav_put_float(buf, 12, MaxTSpd);
    _mav_put_float(buf, 16, DepthPID);
    _mav_put_float(buf, 20, YawPID);
    _mav_put_float(buf, 24, PosCtrl);
    _mav_put_float(buf, 28, AvoiCtrl);
    _mav_put_float(buf, 32, alternate_rov_acoef_float_1);
    _mav_put_float(buf, 36, alternate_rov_acoef_float_2);
    _mav_put_float(buf, 40, alternate_rov_acoef_float_3);
    _mav_put_float(buf, 44, alternate_rov_acoef_float_4);
    _mav_put_float(buf, 48, alternate_rov_acoef_float_5);
    _mav_put_uint32_t(buf, 52, alternate_rov_acoef_int_4);
    _mav_put_uint32_t(buf, 56, alternate_rov_acoef_int_5);
    _mav_put_uint8_t(buf, 60, MovCtrl);
    _mav_put_uint8_t(buf, 61, ServoCtrl);
    _mav_put_uint8_t(buf, 102, alternate_rov_acoef_int_1);
    _mav_put_uint8_t(buf, 103, alternate_rov_acoef_int_2);
    _mav_put_uint8_t(buf, 104, alternate_rov_acoef_int_3);
    _mav_put_char_array(buf, 62, MCtrlIP, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROV_ACOEF, buf, MAVLINK_MSG_ID_ROV_ACOEF_MIN_LEN, MAVLINK_MSG_ID_ROV_ACOEF_LEN, MAVLINK_MSG_ID_ROV_ACOEF_CRC);
#else
    mavlink_rov_acoef_t packet;
    packet.CommuFreq = CommuFreq;
    packet.MaxSAng = MaxSAng;
    packet.MinSAng = MinSAng;
    packet.MaxTSpd = MaxTSpd;
    packet.DepthPID = DepthPID;
    packet.YawPID = YawPID;
    packet.PosCtrl = PosCtrl;
    packet.AvoiCtrl = AvoiCtrl;
    packet.alternate_rov_acoef_float_1 = alternate_rov_acoef_float_1;
    packet.alternate_rov_acoef_float_2 = alternate_rov_acoef_float_2;
    packet.alternate_rov_acoef_float_3 = alternate_rov_acoef_float_3;
    packet.alternate_rov_acoef_float_4 = alternate_rov_acoef_float_4;
    packet.alternate_rov_acoef_float_5 = alternate_rov_acoef_float_5;
    packet.alternate_rov_acoef_int_4 = alternate_rov_acoef_int_4;
    packet.alternate_rov_acoef_int_5 = alternate_rov_acoef_int_5;
    packet.MovCtrl = MovCtrl;
    packet.ServoCtrl = ServoCtrl;
    packet.alternate_rov_acoef_int_1 = alternate_rov_acoef_int_1;
    packet.alternate_rov_acoef_int_2 = alternate_rov_acoef_int_2;
    packet.alternate_rov_acoef_int_3 = alternate_rov_acoef_int_3;
    mav_array_memcpy(packet.MCtrlIP, MCtrlIP, sizeof(char)*40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROV_ACOEF, (const char *)&packet, MAVLINK_MSG_ID_ROV_ACOEF_MIN_LEN, MAVLINK_MSG_ID_ROV_ACOEF_LEN, MAVLINK_MSG_ID_ROV_ACOEF_CRC);
#endif
}

/**
 * @brief Send a rov_acoef message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rov_acoef_send_struct(mavlink_channel_t chan, const mavlink_rov_acoef_t* rov_acoef)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rov_acoef_send(chan, rov_acoef->MovCtrl, rov_acoef->ServoCtrl, rov_acoef->CommuFreq, rov_acoef->MaxSAng, rov_acoef->MinSAng, rov_acoef->MCtrlIP, rov_acoef->MaxTSpd, rov_acoef->DepthPID, rov_acoef->YawPID, rov_acoef->PosCtrl, rov_acoef->AvoiCtrl, rov_acoef->alternate_rov_acoef_float_1, rov_acoef->alternate_rov_acoef_float_2, rov_acoef->alternate_rov_acoef_float_3, rov_acoef->alternate_rov_acoef_float_4, rov_acoef->alternate_rov_acoef_float_5, rov_acoef->alternate_rov_acoef_int_1, rov_acoef->alternate_rov_acoef_int_2, rov_acoef->alternate_rov_acoef_int_3, rov_acoef->alternate_rov_acoef_int_4, rov_acoef->alternate_rov_acoef_int_5);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROV_ACOEF, (const char *)rov_acoef, MAVLINK_MSG_ID_ROV_ACOEF_MIN_LEN, MAVLINK_MSG_ID_ROV_ACOEF_LEN, MAVLINK_MSG_ID_ROV_ACOEF_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROV_ACOEF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rov_acoef_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t MovCtrl, uint8_t ServoCtrl, uint32_t CommuFreq, float MaxSAng, float MinSAng, const char *MCtrlIP, float MaxTSpd, float DepthPID, float YawPID, float PosCtrl, float AvoiCtrl, float alternate_rov_acoef_float_1, float alternate_rov_acoef_float_2, float alternate_rov_acoef_float_3, float alternate_rov_acoef_float_4, float alternate_rov_acoef_float_5, uint8_t alternate_rov_acoef_int_1, uint8_t alternate_rov_acoef_int_2, uint8_t alternate_rov_acoef_int_3, uint32_t alternate_rov_acoef_int_4, uint32_t alternate_rov_acoef_int_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, CommuFreq);
    _mav_put_float(buf, 4, MaxSAng);
    _mav_put_float(buf, 8, MinSAng);
    _mav_put_float(buf, 12, MaxTSpd);
    _mav_put_float(buf, 16, DepthPID);
    _mav_put_float(buf, 20, YawPID);
    _mav_put_float(buf, 24, PosCtrl);
    _mav_put_float(buf, 28, AvoiCtrl);
    _mav_put_float(buf, 32, alternate_rov_acoef_float_1);
    _mav_put_float(buf, 36, alternate_rov_acoef_float_2);
    _mav_put_float(buf, 40, alternate_rov_acoef_float_3);
    _mav_put_float(buf, 44, alternate_rov_acoef_float_4);
    _mav_put_float(buf, 48, alternate_rov_acoef_float_5);
    _mav_put_uint32_t(buf, 52, alternate_rov_acoef_int_4);
    _mav_put_uint32_t(buf, 56, alternate_rov_acoef_int_5);
    _mav_put_uint8_t(buf, 60, MovCtrl);
    _mav_put_uint8_t(buf, 61, ServoCtrl);
    _mav_put_uint8_t(buf, 102, alternate_rov_acoef_int_1);
    _mav_put_uint8_t(buf, 103, alternate_rov_acoef_int_2);
    _mav_put_uint8_t(buf, 104, alternate_rov_acoef_int_3);
    _mav_put_char_array(buf, 62, MCtrlIP, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROV_ACOEF, buf, MAVLINK_MSG_ID_ROV_ACOEF_MIN_LEN, MAVLINK_MSG_ID_ROV_ACOEF_LEN, MAVLINK_MSG_ID_ROV_ACOEF_CRC);
#else
    mavlink_rov_acoef_t *packet = (mavlink_rov_acoef_t *)msgbuf;
    packet->CommuFreq = CommuFreq;
    packet->MaxSAng = MaxSAng;
    packet->MinSAng = MinSAng;
    packet->MaxTSpd = MaxTSpd;
    packet->DepthPID = DepthPID;
    packet->YawPID = YawPID;
    packet->PosCtrl = PosCtrl;
    packet->AvoiCtrl = AvoiCtrl;
    packet->alternate_rov_acoef_float_1 = alternate_rov_acoef_float_1;
    packet->alternate_rov_acoef_float_2 = alternate_rov_acoef_float_2;
    packet->alternate_rov_acoef_float_3 = alternate_rov_acoef_float_3;
    packet->alternate_rov_acoef_float_4 = alternate_rov_acoef_float_4;
    packet->alternate_rov_acoef_float_5 = alternate_rov_acoef_float_5;
    packet->alternate_rov_acoef_int_4 = alternate_rov_acoef_int_4;
    packet->alternate_rov_acoef_int_5 = alternate_rov_acoef_int_5;
    packet->MovCtrl = MovCtrl;
    packet->ServoCtrl = ServoCtrl;
    packet->alternate_rov_acoef_int_1 = alternate_rov_acoef_int_1;
    packet->alternate_rov_acoef_int_2 = alternate_rov_acoef_int_2;
    packet->alternate_rov_acoef_int_3 = alternate_rov_acoef_int_3;
    mav_array_memcpy(packet->MCtrlIP, MCtrlIP, sizeof(char)*40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROV_ACOEF, (const char *)packet, MAVLINK_MSG_ID_ROV_ACOEF_MIN_LEN, MAVLINK_MSG_ID_ROV_ACOEF_LEN, MAVLINK_MSG_ID_ROV_ACOEF_CRC);
#endif
}
#endif

#endif

// MESSAGE ROV_ACOEF UNPACKING


/**
 * @brief Get field MovCtrl from rov_acoef message
 *
 * @return  ON/OFFs-motion control thread.
 */
static inline uint8_t mavlink_msg_rov_acoef_get_MovCtrl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field ServoCtrl from rov_acoef message
 *
 * @return  ON/OFFs-servo control thread.
 */
static inline uint8_t mavlink_msg_rov_acoef_get_ServoCtrl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  61);
}

/**
 * @brief Get field CommuFreq from rov_acoef message
 *
 * @return  mainboard communication frequency.
 */
static inline uint32_t mavlink_msg_rov_acoef_get_CommuFreq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field MaxSAng from rov_acoef message
 *
 * @return  Max servo angle.
 */
static inline float mavlink_msg_rov_acoef_get_MaxSAng(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field MinSAng from rov_acoef message
 *
 * @return  Min servo angle.
 */
static inline float mavlink_msg_rov_acoef_get_MinSAng(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field MCtrlIP from rov_acoef message
 *
 * @return  IP addr. of motion controller.
 */
static inline uint16_t mavlink_msg_rov_acoef_get_MCtrlIP(const mavlink_message_t* msg, char *MCtrlIP)
{
    return _MAV_RETURN_char_array(msg, MCtrlIP, 40,  62);
}

/**
 * @brief Get field MaxTSpd from rov_acoef message
 *
 * @return  Maximum thruster speed.
 */
static inline float mavlink_msg_rov_acoef_get_MaxTSpd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field DepthPID from rov_acoef message
 *
 * @return  Depth PID control coefficients.
 */
static inline float mavlink_msg_rov_acoef_get_DepthPID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field YawPID from rov_acoef message
 *
 * @return  Yaw PID control coefficients.
 */
static inline float mavlink_msg_rov_acoef_get_YawPID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field PosCtrl from rov_acoef message
 *
 * @return  Position control coefficients.
 */
static inline float mavlink_msg_rov_acoef_get_PosCtrl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field AvoiCtrl from rov_acoef message
 *
 * @return  Obstacle avoidance coefficients.
 */
static inline float mavlink_msg_rov_acoef_get_AvoiCtrl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field alternate_rov_acoef_float_1 from rov_acoef message
 *
 * @return  alternate_rov_acoef_float_1.
 */
static inline float mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field alternate_rov_acoef_float_2 from rov_acoef message
 *
 * @return  alternate_rov_acoef_float_2.
 */
static inline float mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field alternate_rov_acoef_float_3 from rov_acoef message
 *
 * @return  alternate_rov_acoef_float_3.
 */
static inline float mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field alternate_rov_acoef_float_4 from rov_acoef message
 *
 * @return  alternate_rov_acoef_float_4.
 */
static inline float mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field alternate_rov_acoef_float_5 from rov_acoef message
 *
 * @return  alternate_rov_acoef_float_5.
 */
static inline float mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field alternate_rov_acoef_int_1 from rov_acoef message
 *
 * @return  alternate_rov_acoef_int_1.
 */
static inline uint8_t mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  102);
}

/**
 * @brief Get field alternate_rov_acoef_int_2 from rov_acoef message
 *
 * @return  alternate_rov_acoef_int_2.
 */
static inline uint8_t mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  103);
}

/**
 * @brief Get field alternate_rov_acoef_int_3 from rov_acoef message
 *
 * @return  alternate_rov_acoef_int_3.
 */
static inline uint8_t mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  104);
}

/**
 * @brief Get field alternate_rov_acoef_int_4 from rov_acoef message
 *
 * @return  alternate_rov_acoef_int_4.
 */
static inline uint32_t mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  52);
}

/**
 * @brief Get field alternate_rov_acoef_int_5 from rov_acoef message
 *
 * @return  alternate_rov_acoef_int_5.
 */
static inline uint32_t mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  56);
}

/**
 * @brief Decode a rov_acoef message into a struct
 *
 * @param msg The message to decode
 * @param rov_acoef C-struct to decode the message contents into
 */
static inline void mavlink_msg_rov_acoef_decode(const mavlink_message_t* msg, mavlink_rov_acoef_t* rov_acoef)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rov_acoef->CommuFreq = mavlink_msg_rov_acoef_get_CommuFreq(msg);
    rov_acoef->MaxSAng = mavlink_msg_rov_acoef_get_MaxSAng(msg);
    rov_acoef->MinSAng = mavlink_msg_rov_acoef_get_MinSAng(msg);
    rov_acoef->MaxTSpd = mavlink_msg_rov_acoef_get_MaxTSpd(msg);
    rov_acoef->DepthPID = mavlink_msg_rov_acoef_get_DepthPID(msg);
    rov_acoef->YawPID = mavlink_msg_rov_acoef_get_YawPID(msg);
    rov_acoef->PosCtrl = mavlink_msg_rov_acoef_get_PosCtrl(msg);
    rov_acoef->AvoiCtrl = mavlink_msg_rov_acoef_get_AvoiCtrl(msg);
    rov_acoef->alternate_rov_acoef_float_1 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_1(msg);
    rov_acoef->alternate_rov_acoef_float_2 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_2(msg);
    rov_acoef->alternate_rov_acoef_float_3 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_3(msg);
    rov_acoef->alternate_rov_acoef_float_4 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_4(msg);
    rov_acoef->alternate_rov_acoef_float_5 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_float_5(msg);
    rov_acoef->alternate_rov_acoef_int_4 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_4(msg);
    rov_acoef->alternate_rov_acoef_int_5 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_5(msg);
    rov_acoef->MovCtrl = mavlink_msg_rov_acoef_get_MovCtrl(msg);
    rov_acoef->ServoCtrl = mavlink_msg_rov_acoef_get_ServoCtrl(msg);
    mavlink_msg_rov_acoef_get_MCtrlIP(msg, rov_acoef->MCtrlIP);
    rov_acoef->alternate_rov_acoef_int_1 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_1(msg);
    rov_acoef->alternate_rov_acoef_int_2 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_2(msg);
    rov_acoef->alternate_rov_acoef_int_3 = mavlink_msg_rov_acoef_get_alternate_rov_acoef_int_3(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROV_ACOEF_LEN? msg->len : MAVLINK_MSG_ID_ROV_ACOEF_LEN;
        memset(rov_acoef, 0, MAVLINK_MSG_ID_ROV_ACOEF_LEN);
    memcpy(rov_acoef, _MAV_PAYLOAD(msg), len);
#endif
}
