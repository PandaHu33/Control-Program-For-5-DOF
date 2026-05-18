#pragma once
// MESSAGE QGC2MANIPULATOR PACKING

#define MAVLINK_MSG_ID_QGC2MANIPULATOR 238


typedef struct __mavlink_qgc2manipulator_t {
 float angle1; /*<  Joint angle1.*/
 float angle2; /*<  Joint angle2.*/
 float angle3; /*<  Joint angle3.*/
 float angle4; /*<  Joint angle4.*/
 float angle5; /*<  Joint angle5.*/
 float angular_vel1; /*<  Joint angular velocity1.*/
 float angular_vel2; /*<  Joint angular velocity2.*/
 float angular_vel3; /*<  Joint angular velocity3.*/
 float angular_vel4; /*<  Joint angular velocity4.*/
 float angular_vel5; /*<  Joint angular velocity5.*/
 float pos_x; /*<  arm x position.*/
 float pos_y; /*<  arm y position.*/
 float pos_z; /*<  arm z position.*/
 float button; /*<  teleoperation button.*/
 float keyboard1; /*<  keyboard setting1.*/
 float keyboard2; /*<  keyboard setting2.*/
 float keyboard3; /*<  keyboard setting3.*/
 float keyboard4; /*<  keyboard setting4.*/
 float keyboard5; /*<  keyboard setting5.*/
 float keyboard6; /*<  keyboard setting6.*/
 float keyboard7; /*<  keyboard setting7.*/
 float keyboard8; /*<  keyboard setting8.*/
 float keyboard9; /*<  keyboard setting9.*/
 float keyboard10; /*<  keyboard setting10.*/
 float status; /*<  manipulator status.*/
 float stop_sign; /*<  emergency stop sign.*/
 float reserved1; /*<  reserved1.*/
 float reserved2; /*<  reserved2.*/
 float reserved3; /*<  reserved3.*/
 float reserved4; /*<  reserved4.*/
} mavlink_qgc2manipulator_t;

#define MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN 120
#define MAVLINK_MSG_ID_QGC2MANIPULATOR_MIN_LEN 120
#define MAVLINK_MSG_ID_238_LEN 120
#define MAVLINK_MSG_ID_238_MIN_LEN 120

#define MAVLINK_MSG_ID_QGC2MANIPULATOR_CRC 217
#define MAVLINK_MSG_ID_238_CRC 217



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_QGC2MANIPULATOR { \
    238, \
    "QGC2MANIPULATOR", \
    30, \
    {  { "angle1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_qgc2manipulator_t, angle1) }, \
         { "angle2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_qgc2manipulator_t, angle2) }, \
         { "angle3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_qgc2manipulator_t, angle3) }, \
         { "angle4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_qgc2manipulator_t, angle4) }, \
         { "angle5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_qgc2manipulator_t, angle5) }, \
         { "angular_vel1", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_qgc2manipulator_t, angular_vel1) }, \
         { "angular_vel2", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_qgc2manipulator_t, angular_vel2) }, \
         { "angular_vel3", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_qgc2manipulator_t, angular_vel3) }, \
         { "angular_vel4", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_qgc2manipulator_t, angular_vel4) }, \
         { "angular_vel5", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_qgc2manipulator_t, angular_vel5) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_qgc2manipulator_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_qgc2manipulator_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_qgc2manipulator_t, pos_z) }, \
         { "button", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_qgc2manipulator_t, button) }, \
         { "keyboard1", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_qgc2manipulator_t, keyboard1) }, \
         { "keyboard2", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_qgc2manipulator_t, keyboard2) }, \
         { "keyboard3", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_qgc2manipulator_t, keyboard3) }, \
         { "keyboard4", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_qgc2manipulator_t, keyboard4) }, \
         { "keyboard5", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_qgc2manipulator_t, keyboard5) }, \
         { "keyboard6", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_qgc2manipulator_t, keyboard6) }, \
         { "keyboard7", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_qgc2manipulator_t, keyboard7) }, \
         { "keyboard8", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_qgc2manipulator_t, keyboard8) }, \
         { "keyboard9", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_qgc2manipulator_t, keyboard9) }, \
         { "keyboard10", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_qgc2manipulator_t, keyboard10) }, \
         { "status", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_qgc2manipulator_t, status) }, \
         { "stop_sign", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_qgc2manipulator_t, stop_sign) }, \
         { "reserved1", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_qgc2manipulator_t, reserved1) }, \
         { "reserved2", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_qgc2manipulator_t, reserved2) }, \
         { "reserved3", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_qgc2manipulator_t, reserved3) }, \
         { "reserved4", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_qgc2manipulator_t, reserved4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_QGC2MANIPULATOR { \
    "QGC2MANIPULATOR", \
    30, \
    {  { "angle1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_qgc2manipulator_t, angle1) }, \
         { "angle2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_qgc2manipulator_t, angle2) }, \
         { "angle3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_qgc2manipulator_t, angle3) }, \
         { "angle4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_qgc2manipulator_t, angle4) }, \
         { "angle5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_qgc2manipulator_t, angle5) }, \
         { "angular_vel1", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_qgc2manipulator_t, angular_vel1) }, \
         { "angular_vel2", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_qgc2manipulator_t, angular_vel2) }, \
         { "angular_vel3", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_qgc2manipulator_t, angular_vel3) }, \
         { "angular_vel4", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_qgc2manipulator_t, angular_vel4) }, \
         { "angular_vel5", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_qgc2manipulator_t, angular_vel5) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_qgc2manipulator_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_qgc2manipulator_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_qgc2manipulator_t, pos_z) }, \
         { "button", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_qgc2manipulator_t, button) }, \
         { "keyboard1", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_qgc2manipulator_t, keyboard1) }, \
         { "keyboard2", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_qgc2manipulator_t, keyboard2) }, \
         { "keyboard3", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_qgc2manipulator_t, keyboard3) }, \
         { "keyboard4", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_qgc2manipulator_t, keyboard4) }, \
         { "keyboard5", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_qgc2manipulator_t, keyboard5) }, \
         { "keyboard6", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_qgc2manipulator_t, keyboard6) }, \
         { "keyboard7", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_qgc2manipulator_t, keyboard7) }, \
         { "keyboard8", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_qgc2manipulator_t, keyboard8) }, \
         { "keyboard9", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_qgc2manipulator_t, keyboard9) }, \
         { "keyboard10", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_qgc2manipulator_t, keyboard10) }, \
         { "status", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_qgc2manipulator_t, status) }, \
         { "stop_sign", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_qgc2manipulator_t, stop_sign) }, \
         { "reserved1", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_qgc2manipulator_t, reserved1) }, \
         { "reserved2", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_qgc2manipulator_t, reserved2) }, \
         { "reserved3", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_qgc2manipulator_t, reserved3) }, \
         { "reserved4", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_qgc2manipulator_t, reserved4) }, \
         } \
}
#endif

/**
 * @brief Pack a qgc2manipulator message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param angle1  Joint angle1.
 * @param angle2  Joint angle2.
 * @param angle3  Joint angle3.
 * @param angle4  Joint angle4.
 * @param angle5  Joint angle5.
 * @param angular_vel1  Joint angular velocity1.
 * @param angular_vel2  Joint angular velocity2.
 * @param angular_vel3  Joint angular velocity3.
 * @param angular_vel4  Joint angular velocity4.
 * @param angular_vel5  Joint angular velocity5.
 * @param pos_x  arm x position.
 * @param pos_y  arm y position.
 * @param pos_z  arm z position.
 * @param button  teleoperation button.
 * @param keyboard1  keyboard setting1.
 * @param keyboard2  keyboard setting2.
 * @param keyboard3  keyboard setting3.
 * @param keyboard4  keyboard setting4.
 * @param keyboard5  keyboard setting5.
 * @param keyboard6  keyboard setting6.
 * @param keyboard7  keyboard setting7.
 * @param keyboard8  keyboard setting8.
 * @param keyboard9  keyboard setting9.
 * @param keyboard10  keyboard setting10.
 * @param status  manipulator status.
 * @param stop_sign  emergency stop sign.
 * @param reserved1  reserved1.
 * @param reserved2  reserved2.
 * @param reserved3  reserved3.
 * @param reserved4  reserved4.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_qgc2manipulator_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float angle1, float angle2, float angle3, float angle4, float angle5, float angular_vel1, float angular_vel2, float angular_vel3, float angular_vel4, float angular_vel5, float pos_x, float pos_y, float pos_z, float button, float keyboard1, float keyboard2, float keyboard3, float keyboard4, float keyboard5, float keyboard6, float keyboard7, float keyboard8, float keyboard9, float keyboard10, float status, float stop_sign, float reserved1, float reserved2, float reserved3, float reserved4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN];
    _mav_put_float(buf, 0, angle1);
    _mav_put_float(buf, 4, angle2);
    _mav_put_float(buf, 8, angle3);
    _mav_put_float(buf, 12, angle4);
    _mav_put_float(buf, 16, angle5);
    _mav_put_float(buf, 20, angular_vel1);
    _mav_put_float(buf, 24, angular_vel2);
    _mav_put_float(buf, 28, angular_vel3);
    _mav_put_float(buf, 32, angular_vel4);
    _mav_put_float(buf, 36, angular_vel5);
    _mav_put_float(buf, 40, pos_x);
    _mav_put_float(buf, 44, pos_y);
    _mav_put_float(buf, 48, pos_z);
    _mav_put_float(buf, 52, button);
    _mav_put_float(buf, 56, keyboard1);
    _mav_put_float(buf, 60, keyboard2);
    _mav_put_float(buf, 64, keyboard3);
    _mav_put_float(buf, 68, keyboard4);
    _mav_put_float(buf, 72, keyboard5);
    _mav_put_float(buf, 76, keyboard6);
    _mav_put_float(buf, 80, keyboard7);
    _mav_put_float(buf, 84, keyboard8);
    _mav_put_float(buf, 88, keyboard9);
    _mav_put_float(buf, 92, keyboard10);
    _mav_put_float(buf, 96, status);
    _mav_put_float(buf, 100, stop_sign);
    _mav_put_float(buf, 104, reserved1);
    _mav_put_float(buf, 108, reserved2);
    _mav_put_float(buf, 112, reserved3);
    _mav_put_float(buf, 116, reserved4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN);
#else
    mavlink_qgc2manipulator_t packet;
    packet.angle1 = angle1;
    packet.angle2 = angle2;
    packet.angle3 = angle3;
    packet.angle4 = angle4;
    packet.angle5 = angle5;
    packet.angular_vel1 = angular_vel1;
    packet.angular_vel2 = angular_vel2;
    packet.angular_vel3 = angular_vel3;
    packet.angular_vel4 = angular_vel4;
    packet.angular_vel5 = angular_vel5;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.button = button;
    packet.keyboard1 = keyboard1;
    packet.keyboard2 = keyboard2;
    packet.keyboard3 = keyboard3;
    packet.keyboard4 = keyboard4;
    packet.keyboard5 = keyboard5;
    packet.keyboard6 = keyboard6;
    packet.keyboard7 = keyboard7;
    packet.keyboard8 = keyboard8;
    packet.keyboard9 = keyboard9;
    packet.keyboard10 = keyboard10;
    packet.status = status;
    packet.stop_sign = stop_sign;
    packet.reserved1 = reserved1;
    packet.reserved2 = reserved2;
    packet.reserved3 = reserved3;
    packet.reserved4 = reserved4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QGC2MANIPULATOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QGC2MANIPULATOR_MIN_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_CRC);
}

/**
 * @brief Pack a qgc2manipulator message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param angle1  Joint angle1.
 * @param angle2  Joint angle2.
 * @param angle3  Joint angle3.
 * @param angle4  Joint angle4.
 * @param angle5  Joint angle5.
 * @param angular_vel1  Joint angular velocity1.
 * @param angular_vel2  Joint angular velocity2.
 * @param angular_vel3  Joint angular velocity3.
 * @param angular_vel4  Joint angular velocity4.
 * @param angular_vel5  Joint angular velocity5.
 * @param pos_x  arm x position.
 * @param pos_y  arm y position.
 * @param pos_z  arm z position.
 * @param button  teleoperation button.
 * @param keyboard1  keyboard setting1.
 * @param keyboard2  keyboard setting2.
 * @param keyboard3  keyboard setting3.
 * @param keyboard4  keyboard setting4.
 * @param keyboard5  keyboard setting5.
 * @param keyboard6  keyboard setting6.
 * @param keyboard7  keyboard setting7.
 * @param keyboard8  keyboard setting8.
 * @param keyboard9  keyboard setting9.
 * @param keyboard10  keyboard setting10.
 * @param status  manipulator status.
 * @param stop_sign  emergency stop sign.
 * @param reserved1  reserved1.
 * @param reserved2  reserved2.
 * @param reserved3  reserved3.
 * @param reserved4  reserved4.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_qgc2manipulator_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float angle1,float angle2,float angle3,float angle4,float angle5,float angular_vel1,float angular_vel2,float angular_vel3,float angular_vel4,float angular_vel5,float pos_x,float pos_y,float pos_z,float button,float keyboard1,float keyboard2,float keyboard3,float keyboard4,float keyboard5,float keyboard6,float keyboard7,float keyboard8,float keyboard9,float keyboard10,float status,float stop_sign,float reserved1,float reserved2,float reserved3,float reserved4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN];
    _mav_put_float(buf, 0, angle1);
    _mav_put_float(buf, 4, angle2);
    _mav_put_float(buf, 8, angle3);
    _mav_put_float(buf, 12, angle4);
    _mav_put_float(buf, 16, angle5);
    _mav_put_float(buf, 20, angular_vel1);
    _mav_put_float(buf, 24, angular_vel2);
    _mav_put_float(buf, 28, angular_vel3);
    _mav_put_float(buf, 32, angular_vel4);
    _mav_put_float(buf, 36, angular_vel5);
    _mav_put_float(buf, 40, pos_x);
    _mav_put_float(buf, 44, pos_y);
    _mav_put_float(buf, 48, pos_z);
    _mav_put_float(buf, 52, button);
    _mav_put_float(buf, 56, keyboard1);
    _mav_put_float(buf, 60, keyboard2);
    _mav_put_float(buf, 64, keyboard3);
    _mav_put_float(buf, 68, keyboard4);
    _mav_put_float(buf, 72, keyboard5);
    _mav_put_float(buf, 76, keyboard6);
    _mav_put_float(buf, 80, keyboard7);
    _mav_put_float(buf, 84, keyboard8);
    _mav_put_float(buf, 88, keyboard9);
    _mav_put_float(buf, 92, keyboard10);
    _mav_put_float(buf, 96, status);
    _mav_put_float(buf, 100, stop_sign);
    _mav_put_float(buf, 104, reserved1);
    _mav_put_float(buf, 108, reserved2);
    _mav_put_float(buf, 112, reserved3);
    _mav_put_float(buf, 116, reserved4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN);
#else
    mavlink_qgc2manipulator_t packet;
    packet.angle1 = angle1;
    packet.angle2 = angle2;
    packet.angle3 = angle3;
    packet.angle4 = angle4;
    packet.angle5 = angle5;
    packet.angular_vel1 = angular_vel1;
    packet.angular_vel2 = angular_vel2;
    packet.angular_vel3 = angular_vel3;
    packet.angular_vel4 = angular_vel4;
    packet.angular_vel5 = angular_vel5;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.button = button;
    packet.keyboard1 = keyboard1;
    packet.keyboard2 = keyboard2;
    packet.keyboard3 = keyboard3;
    packet.keyboard4 = keyboard4;
    packet.keyboard5 = keyboard5;
    packet.keyboard6 = keyboard6;
    packet.keyboard7 = keyboard7;
    packet.keyboard8 = keyboard8;
    packet.keyboard9 = keyboard9;
    packet.keyboard10 = keyboard10;
    packet.status = status;
    packet.stop_sign = stop_sign;
    packet.reserved1 = reserved1;
    packet.reserved2 = reserved2;
    packet.reserved3 = reserved3;
    packet.reserved4 = reserved4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QGC2MANIPULATOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QGC2MANIPULATOR_MIN_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_CRC);
}

/**
 * @brief Encode a qgc2manipulator struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param qgc2manipulator C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_qgc2manipulator_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_qgc2manipulator_t* qgc2manipulator)
{
    return mavlink_msg_qgc2manipulator_pack(system_id, component_id, msg, qgc2manipulator->angle1, qgc2manipulator->angle2, qgc2manipulator->angle3, qgc2manipulator->angle4, qgc2manipulator->angle5, qgc2manipulator->angular_vel1, qgc2manipulator->angular_vel2, qgc2manipulator->angular_vel3, qgc2manipulator->angular_vel4, qgc2manipulator->angular_vel5, qgc2manipulator->pos_x, qgc2manipulator->pos_y, qgc2manipulator->pos_z, qgc2manipulator->button, qgc2manipulator->keyboard1, qgc2manipulator->keyboard2, qgc2manipulator->keyboard3, qgc2manipulator->keyboard4, qgc2manipulator->keyboard5, qgc2manipulator->keyboard6, qgc2manipulator->keyboard7, qgc2manipulator->keyboard8, qgc2manipulator->keyboard9, qgc2manipulator->keyboard10, qgc2manipulator->status, qgc2manipulator->stop_sign, qgc2manipulator->reserved1, qgc2manipulator->reserved2, qgc2manipulator->reserved3, qgc2manipulator->reserved4);
}

/**
 * @brief Encode a qgc2manipulator struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param qgc2manipulator C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_qgc2manipulator_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_qgc2manipulator_t* qgc2manipulator)
{
    return mavlink_msg_qgc2manipulator_pack_chan(system_id, component_id, chan, msg, qgc2manipulator->angle1, qgc2manipulator->angle2, qgc2manipulator->angle3, qgc2manipulator->angle4, qgc2manipulator->angle5, qgc2manipulator->angular_vel1, qgc2manipulator->angular_vel2, qgc2manipulator->angular_vel3, qgc2manipulator->angular_vel4, qgc2manipulator->angular_vel5, qgc2manipulator->pos_x, qgc2manipulator->pos_y, qgc2manipulator->pos_z, qgc2manipulator->button, qgc2manipulator->keyboard1, qgc2manipulator->keyboard2, qgc2manipulator->keyboard3, qgc2manipulator->keyboard4, qgc2manipulator->keyboard5, qgc2manipulator->keyboard6, qgc2manipulator->keyboard7, qgc2manipulator->keyboard8, qgc2manipulator->keyboard9, qgc2manipulator->keyboard10, qgc2manipulator->status, qgc2manipulator->stop_sign, qgc2manipulator->reserved1, qgc2manipulator->reserved2, qgc2manipulator->reserved3, qgc2manipulator->reserved4);
}

/**
 * @brief Send a qgc2manipulator message
 * @param chan MAVLink channel to send the message
 *
 * @param angle1  Joint angle1.
 * @param angle2  Joint angle2.
 * @param angle3  Joint angle3.
 * @param angle4  Joint angle4.
 * @param angle5  Joint angle5.
 * @param angular_vel1  Joint angular velocity1.
 * @param angular_vel2  Joint angular velocity2.
 * @param angular_vel3  Joint angular velocity3.
 * @param angular_vel4  Joint angular velocity4.
 * @param angular_vel5  Joint angular velocity5.
 * @param pos_x  arm x position.
 * @param pos_y  arm y position.
 * @param pos_z  arm z position.
 * @param button  teleoperation button.
 * @param keyboard1  keyboard setting1.
 * @param keyboard2  keyboard setting2.
 * @param keyboard3  keyboard setting3.
 * @param keyboard4  keyboard setting4.
 * @param keyboard5  keyboard setting5.
 * @param keyboard6  keyboard setting6.
 * @param keyboard7  keyboard setting7.
 * @param keyboard8  keyboard setting8.
 * @param keyboard9  keyboard setting9.
 * @param keyboard10  keyboard setting10.
 * @param status  manipulator status.
 * @param stop_sign  emergency stop sign.
 * @param reserved1  reserved1.
 * @param reserved2  reserved2.
 * @param reserved3  reserved3.
 * @param reserved4  reserved4.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_qgc2manipulator_send(mavlink_channel_t chan, float angle1, float angle2, float angle3, float angle4, float angle5, float angular_vel1, float angular_vel2, float angular_vel3, float angular_vel4, float angular_vel5, float pos_x, float pos_y, float pos_z, float button, float keyboard1, float keyboard2, float keyboard3, float keyboard4, float keyboard5, float keyboard6, float keyboard7, float keyboard8, float keyboard9, float keyboard10, float status, float stop_sign, float reserved1, float reserved2, float reserved3, float reserved4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN];
    _mav_put_float(buf, 0, angle1);
    _mav_put_float(buf, 4, angle2);
    _mav_put_float(buf, 8, angle3);
    _mav_put_float(buf, 12, angle4);
    _mav_put_float(buf, 16, angle5);
    _mav_put_float(buf, 20, angular_vel1);
    _mav_put_float(buf, 24, angular_vel2);
    _mav_put_float(buf, 28, angular_vel3);
    _mav_put_float(buf, 32, angular_vel4);
    _mav_put_float(buf, 36, angular_vel5);
    _mav_put_float(buf, 40, pos_x);
    _mav_put_float(buf, 44, pos_y);
    _mav_put_float(buf, 48, pos_z);
    _mav_put_float(buf, 52, button);
    _mav_put_float(buf, 56, keyboard1);
    _mav_put_float(buf, 60, keyboard2);
    _mav_put_float(buf, 64, keyboard3);
    _mav_put_float(buf, 68, keyboard4);
    _mav_put_float(buf, 72, keyboard5);
    _mav_put_float(buf, 76, keyboard6);
    _mav_put_float(buf, 80, keyboard7);
    _mav_put_float(buf, 84, keyboard8);
    _mav_put_float(buf, 88, keyboard9);
    _mav_put_float(buf, 92, keyboard10);
    _mav_put_float(buf, 96, status);
    _mav_put_float(buf, 100, stop_sign);
    _mav_put_float(buf, 104, reserved1);
    _mav_put_float(buf, 108, reserved2);
    _mav_put_float(buf, 112, reserved3);
    _mav_put_float(buf, 116, reserved4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QGC2MANIPULATOR, buf, MAVLINK_MSG_ID_QGC2MANIPULATOR_MIN_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_CRC);
#else
    mavlink_qgc2manipulator_t packet;
    packet.angle1 = angle1;
    packet.angle2 = angle2;
    packet.angle3 = angle3;
    packet.angle4 = angle4;
    packet.angle5 = angle5;
    packet.angular_vel1 = angular_vel1;
    packet.angular_vel2 = angular_vel2;
    packet.angular_vel3 = angular_vel3;
    packet.angular_vel4 = angular_vel4;
    packet.angular_vel5 = angular_vel5;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.button = button;
    packet.keyboard1 = keyboard1;
    packet.keyboard2 = keyboard2;
    packet.keyboard3 = keyboard3;
    packet.keyboard4 = keyboard4;
    packet.keyboard5 = keyboard5;
    packet.keyboard6 = keyboard6;
    packet.keyboard7 = keyboard7;
    packet.keyboard8 = keyboard8;
    packet.keyboard9 = keyboard9;
    packet.keyboard10 = keyboard10;
    packet.status = status;
    packet.stop_sign = stop_sign;
    packet.reserved1 = reserved1;
    packet.reserved2 = reserved2;
    packet.reserved3 = reserved3;
    packet.reserved4 = reserved4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QGC2MANIPULATOR, (const char *)&packet, MAVLINK_MSG_ID_QGC2MANIPULATOR_MIN_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_CRC);
#endif
}

/**
 * @brief Send a qgc2manipulator message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_qgc2manipulator_send_struct(mavlink_channel_t chan, const mavlink_qgc2manipulator_t* qgc2manipulator)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_qgc2manipulator_send(chan, qgc2manipulator->angle1, qgc2manipulator->angle2, qgc2manipulator->angle3, qgc2manipulator->angle4, qgc2manipulator->angle5, qgc2manipulator->angular_vel1, qgc2manipulator->angular_vel2, qgc2manipulator->angular_vel3, qgc2manipulator->angular_vel4, qgc2manipulator->angular_vel5, qgc2manipulator->pos_x, qgc2manipulator->pos_y, qgc2manipulator->pos_z, qgc2manipulator->button, qgc2manipulator->keyboard1, qgc2manipulator->keyboard2, qgc2manipulator->keyboard3, qgc2manipulator->keyboard4, qgc2manipulator->keyboard5, qgc2manipulator->keyboard6, qgc2manipulator->keyboard7, qgc2manipulator->keyboard8, qgc2manipulator->keyboard9, qgc2manipulator->keyboard10, qgc2manipulator->status, qgc2manipulator->stop_sign, qgc2manipulator->reserved1, qgc2manipulator->reserved2, qgc2manipulator->reserved3, qgc2manipulator->reserved4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QGC2MANIPULATOR, (const char *)qgc2manipulator, MAVLINK_MSG_ID_QGC2MANIPULATOR_MIN_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_qgc2manipulator_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float angle1, float angle2, float angle3, float angle4, float angle5, float angular_vel1, float angular_vel2, float angular_vel3, float angular_vel4, float angular_vel5, float pos_x, float pos_y, float pos_z, float button, float keyboard1, float keyboard2, float keyboard3, float keyboard4, float keyboard5, float keyboard6, float keyboard7, float keyboard8, float keyboard9, float keyboard10, float status, float stop_sign, float reserved1, float reserved2, float reserved3, float reserved4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, angle1);
    _mav_put_float(buf, 4, angle2);
    _mav_put_float(buf, 8, angle3);
    _mav_put_float(buf, 12, angle4);
    _mav_put_float(buf, 16, angle5);
    _mav_put_float(buf, 20, angular_vel1);
    _mav_put_float(buf, 24, angular_vel2);
    _mav_put_float(buf, 28, angular_vel3);
    _mav_put_float(buf, 32, angular_vel4);
    _mav_put_float(buf, 36, angular_vel5);
    _mav_put_float(buf, 40, pos_x);
    _mav_put_float(buf, 44, pos_y);
    _mav_put_float(buf, 48, pos_z);
    _mav_put_float(buf, 52, button);
    _mav_put_float(buf, 56, keyboard1);
    _mav_put_float(buf, 60, keyboard2);
    _mav_put_float(buf, 64, keyboard3);
    _mav_put_float(buf, 68, keyboard4);
    _mav_put_float(buf, 72, keyboard5);
    _mav_put_float(buf, 76, keyboard6);
    _mav_put_float(buf, 80, keyboard7);
    _mav_put_float(buf, 84, keyboard8);
    _mav_put_float(buf, 88, keyboard9);
    _mav_put_float(buf, 92, keyboard10);
    _mav_put_float(buf, 96, status);
    _mav_put_float(buf, 100, stop_sign);
    _mav_put_float(buf, 104, reserved1);
    _mav_put_float(buf, 108, reserved2);
    _mav_put_float(buf, 112, reserved3);
    _mav_put_float(buf, 116, reserved4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QGC2MANIPULATOR, buf, MAVLINK_MSG_ID_QGC2MANIPULATOR_MIN_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_CRC);
#else
    mavlink_qgc2manipulator_t *packet = (mavlink_qgc2manipulator_t *)msgbuf;
    packet->angle1 = angle1;
    packet->angle2 = angle2;
    packet->angle3 = angle3;
    packet->angle4 = angle4;
    packet->angle5 = angle5;
    packet->angular_vel1 = angular_vel1;
    packet->angular_vel2 = angular_vel2;
    packet->angular_vel3 = angular_vel3;
    packet->angular_vel4 = angular_vel4;
    packet->angular_vel5 = angular_vel5;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;
    packet->pos_z = pos_z;
    packet->button = button;
    packet->keyboard1 = keyboard1;
    packet->keyboard2 = keyboard2;
    packet->keyboard3 = keyboard3;
    packet->keyboard4 = keyboard4;
    packet->keyboard5 = keyboard5;
    packet->keyboard6 = keyboard6;
    packet->keyboard7 = keyboard7;
    packet->keyboard8 = keyboard8;
    packet->keyboard9 = keyboard9;
    packet->keyboard10 = keyboard10;
    packet->status = status;
    packet->stop_sign = stop_sign;
    packet->reserved1 = reserved1;
    packet->reserved2 = reserved2;
    packet->reserved3 = reserved3;
    packet->reserved4 = reserved4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QGC2MANIPULATOR, (const char *)packet, MAVLINK_MSG_ID_QGC2MANIPULATOR_MIN_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN, MAVLINK_MSG_ID_QGC2MANIPULATOR_CRC);
#endif
}
#endif

#endif

// MESSAGE QGC2MANIPULATOR UNPACKING


/**
 * @brief Get field angle1 from qgc2manipulator message
 *
 * @return  Joint angle1.
 */
static inline float mavlink_msg_qgc2manipulator_get_angle1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field angle2 from qgc2manipulator message
 *
 * @return  Joint angle2.
 */
static inline float mavlink_msg_qgc2manipulator_get_angle2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field angle3 from qgc2manipulator message
 *
 * @return  Joint angle3.
 */
static inline float mavlink_msg_qgc2manipulator_get_angle3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field angle4 from qgc2manipulator message
 *
 * @return  Joint angle4.
 */
static inline float mavlink_msg_qgc2manipulator_get_angle4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field angle5 from qgc2manipulator message
 *
 * @return  Joint angle5.
 */
static inline float mavlink_msg_qgc2manipulator_get_angle5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field angular_vel1 from qgc2manipulator message
 *
 * @return  Joint angular velocity1.
 */
static inline float mavlink_msg_qgc2manipulator_get_angular_vel1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field angular_vel2 from qgc2manipulator message
 *
 * @return  Joint angular velocity2.
 */
static inline float mavlink_msg_qgc2manipulator_get_angular_vel2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field angular_vel3 from qgc2manipulator message
 *
 * @return  Joint angular velocity3.
 */
static inline float mavlink_msg_qgc2manipulator_get_angular_vel3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field angular_vel4 from qgc2manipulator message
 *
 * @return  Joint angular velocity4.
 */
static inline float mavlink_msg_qgc2manipulator_get_angular_vel4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field angular_vel5 from qgc2manipulator message
 *
 * @return  Joint angular velocity5.
 */
static inline float mavlink_msg_qgc2manipulator_get_angular_vel5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field pos_x from qgc2manipulator message
 *
 * @return  arm x position.
 */
static inline float mavlink_msg_qgc2manipulator_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field pos_y from qgc2manipulator message
 *
 * @return  arm y position.
 */
static inline float mavlink_msg_qgc2manipulator_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field pos_z from qgc2manipulator message
 *
 * @return  arm z position.
 */
static inline float mavlink_msg_qgc2manipulator_get_pos_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field button from qgc2manipulator message
 *
 * @return  teleoperation button.
 */
static inline float mavlink_msg_qgc2manipulator_get_button(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field keyboard1 from qgc2manipulator message
 *
 * @return  keyboard setting1.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field keyboard2 from qgc2manipulator message
 *
 * @return  keyboard setting2.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field keyboard3 from qgc2manipulator message
 *
 * @return  keyboard setting3.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field keyboard4 from qgc2manipulator message
 *
 * @return  keyboard setting4.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field keyboard5 from qgc2manipulator message
 *
 * @return  keyboard setting5.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field keyboard6 from qgc2manipulator message
 *
 * @return  keyboard setting6.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field keyboard7 from qgc2manipulator message
 *
 * @return  keyboard setting7.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field keyboard8 from qgc2manipulator message
 *
 * @return  keyboard setting8.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard8(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field keyboard9 from qgc2manipulator message
 *
 * @return  keyboard setting9.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard9(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field keyboard10 from qgc2manipulator message
 *
 * @return  keyboard setting10.
 */
static inline float mavlink_msg_qgc2manipulator_get_keyboard10(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field status from qgc2manipulator message
 *
 * @return  manipulator status.
 */
static inline float mavlink_msg_qgc2manipulator_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field stop_sign from qgc2manipulator message
 *
 * @return  emergency stop sign.
 */
static inline float mavlink_msg_qgc2manipulator_get_stop_sign(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field reserved1 from qgc2manipulator message
 *
 * @return  reserved1.
 */
static inline float mavlink_msg_qgc2manipulator_get_reserved1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field reserved2 from qgc2manipulator message
 *
 * @return  reserved2.
 */
static inline float mavlink_msg_qgc2manipulator_get_reserved2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field reserved3 from qgc2manipulator message
 *
 * @return  reserved3.
 */
static inline float mavlink_msg_qgc2manipulator_get_reserved3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field reserved4 from qgc2manipulator message
 *
 * @return  reserved4.
 */
static inline float mavlink_msg_qgc2manipulator_get_reserved4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Decode a qgc2manipulator message into a struct
 *
 * @param msg The message to decode
 * @param qgc2manipulator C-struct to decode the message contents into
 */
static inline void mavlink_msg_qgc2manipulator_decode(const mavlink_message_t* msg, mavlink_qgc2manipulator_t* qgc2manipulator)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    qgc2manipulator->angle1 = mavlink_msg_qgc2manipulator_get_angle1(msg);
    qgc2manipulator->angle2 = mavlink_msg_qgc2manipulator_get_angle2(msg);
    qgc2manipulator->angle3 = mavlink_msg_qgc2manipulator_get_angle3(msg);
    qgc2manipulator->angle4 = mavlink_msg_qgc2manipulator_get_angle4(msg);
    qgc2manipulator->angle5 = mavlink_msg_qgc2manipulator_get_angle5(msg);
    qgc2manipulator->angular_vel1 = mavlink_msg_qgc2manipulator_get_angular_vel1(msg);
    qgc2manipulator->angular_vel2 = mavlink_msg_qgc2manipulator_get_angular_vel2(msg);
    qgc2manipulator->angular_vel3 = mavlink_msg_qgc2manipulator_get_angular_vel3(msg);
    qgc2manipulator->angular_vel4 = mavlink_msg_qgc2manipulator_get_angular_vel4(msg);
    qgc2manipulator->angular_vel5 = mavlink_msg_qgc2manipulator_get_angular_vel5(msg);
    qgc2manipulator->pos_x = mavlink_msg_qgc2manipulator_get_pos_x(msg);
    qgc2manipulator->pos_y = mavlink_msg_qgc2manipulator_get_pos_y(msg);
    qgc2manipulator->pos_z = mavlink_msg_qgc2manipulator_get_pos_z(msg);
    qgc2manipulator->button = mavlink_msg_qgc2manipulator_get_button(msg);
    qgc2manipulator->keyboard1 = mavlink_msg_qgc2manipulator_get_keyboard1(msg);
    qgc2manipulator->keyboard2 = mavlink_msg_qgc2manipulator_get_keyboard2(msg);
    qgc2manipulator->keyboard3 = mavlink_msg_qgc2manipulator_get_keyboard3(msg);
    qgc2manipulator->keyboard4 = mavlink_msg_qgc2manipulator_get_keyboard4(msg);
    qgc2manipulator->keyboard5 = mavlink_msg_qgc2manipulator_get_keyboard5(msg);
    qgc2manipulator->keyboard6 = mavlink_msg_qgc2manipulator_get_keyboard6(msg);
    qgc2manipulator->keyboard7 = mavlink_msg_qgc2manipulator_get_keyboard7(msg);
    qgc2manipulator->keyboard8 = mavlink_msg_qgc2manipulator_get_keyboard8(msg);
    qgc2manipulator->keyboard9 = mavlink_msg_qgc2manipulator_get_keyboard9(msg);
    qgc2manipulator->keyboard10 = mavlink_msg_qgc2manipulator_get_keyboard10(msg);
    qgc2manipulator->status = mavlink_msg_qgc2manipulator_get_status(msg);
    qgc2manipulator->stop_sign = mavlink_msg_qgc2manipulator_get_stop_sign(msg);
    qgc2manipulator->reserved1 = mavlink_msg_qgc2manipulator_get_reserved1(msg);
    qgc2manipulator->reserved2 = mavlink_msg_qgc2manipulator_get_reserved2(msg);
    qgc2manipulator->reserved3 = mavlink_msg_qgc2manipulator_get_reserved3(msg);
    qgc2manipulator->reserved4 = mavlink_msg_qgc2manipulator_get_reserved4(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN? msg->len : MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN;
        memset(qgc2manipulator, 0, MAVLINK_MSG_ID_QGC2MANIPULATOR_LEN);
    memcpy(qgc2manipulator, _MAV_PAYLOAD(msg), len);
#endif
}
