#pragma once
// MESSAGE depth_target PACKING

#define MAVLINK_MSG_ID_depth_target 237


typedef struct __mavlink_depth_target_t {
 float byte1; /*<  depth_target.*/
} mavlink_depth_target_t;

#define MAVLINK_MSG_ID_depth_target_LEN 4
#define MAVLINK_MSG_ID_depth_target_MIN_LEN 4
#define MAVLINK_MSG_ID_237_LEN 4
#define MAVLINK_MSG_ID_237_MIN_LEN 4

#define MAVLINK_MSG_ID_depth_target_CRC 101
#define MAVLINK_MSG_ID_237_CRC 101



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_depth_target { \
    237, \
    "depth_target", \
    1, \
    {  { "byte1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_depth_target_t, byte1) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_depth_target { \
    "depth_target", \
    1, \
    {  { "byte1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_depth_target_t, byte1) }, \
         } \
}
#endif

/**
 * @brief Pack a depth_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param byte1  depth_target.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_depth_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float byte1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_depth_target_LEN];
    _mav_put_float(buf, 0, byte1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_depth_target_LEN);
#else
    mavlink_depth_target_t packet;
    packet.byte1 = byte1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_depth_target_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_depth_target;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_depth_target_MIN_LEN, MAVLINK_MSG_ID_depth_target_LEN, MAVLINK_MSG_ID_depth_target_CRC);
}

/**
 * @brief Pack a depth_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param byte1  depth_target.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_depth_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float byte1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_depth_target_LEN];
    _mav_put_float(buf, 0, byte1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_depth_target_LEN);
#else
    mavlink_depth_target_t packet;
    packet.byte1 = byte1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_depth_target_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_depth_target;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_depth_target_MIN_LEN, MAVLINK_MSG_ID_depth_target_LEN, MAVLINK_MSG_ID_depth_target_CRC);
}

/**
 * @brief Encode a depth_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param depth_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_depth_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_depth_target_t* depth_target)
{
    return mavlink_msg_depth_target_pack(system_id, component_id, msg, depth_target->byte1);
}

/**
 * @brief Encode a depth_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param depth_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_depth_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_depth_target_t* depth_target)
{
    return mavlink_msg_depth_target_pack_chan(system_id, component_id, chan, msg, depth_target->byte1);
}

/**
 * @brief Send a depth_target message
 * @param chan MAVLink channel to send the message
 *
 * @param byte1  depth_target.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_depth_target_send(mavlink_channel_t chan, float byte1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_depth_target_LEN];
    _mav_put_float(buf, 0, byte1);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_depth_target, buf, MAVLINK_MSG_ID_depth_target_MIN_LEN, MAVLINK_MSG_ID_depth_target_LEN, MAVLINK_MSG_ID_depth_target_CRC);
#else
    mavlink_depth_target_t packet;
    packet.byte1 = byte1;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_depth_target, (const char *)&packet, MAVLINK_MSG_ID_depth_target_MIN_LEN, MAVLINK_MSG_ID_depth_target_LEN, MAVLINK_MSG_ID_depth_target_CRC);
#endif
}

/**
 * @brief Send a depth_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_depth_target_send_struct(mavlink_channel_t chan, const mavlink_depth_target_t* depth_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_depth_target_send(chan, depth_target->byte1);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_depth_target, (const char *)depth_target, MAVLINK_MSG_ID_depth_target_MIN_LEN, MAVLINK_MSG_ID_depth_target_LEN, MAVLINK_MSG_ID_depth_target_CRC);
#endif
}

#if MAVLINK_MSG_ID_depth_target_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_depth_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float byte1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, byte1);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_depth_target, buf, MAVLINK_MSG_ID_depth_target_MIN_LEN, MAVLINK_MSG_ID_depth_target_LEN, MAVLINK_MSG_ID_depth_target_CRC);
#else
    mavlink_depth_target_t *packet = (mavlink_depth_target_t *)msgbuf;
    packet->byte1 = byte1;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_depth_target, (const char *)packet, MAVLINK_MSG_ID_depth_target_MIN_LEN, MAVLINK_MSG_ID_depth_target_LEN, MAVLINK_MSG_ID_depth_target_CRC);
#endif
}
#endif

#endif

// MESSAGE depth_target UNPACKING


/**
 * @brief Get field byte1 from depth_target message
 *
 * @return  depth_target.
 */
static inline float mavlink_msg_depth_target_get_byte1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a depth_target message into a struct
 *
 * @param msg The message to decode
 * @param depth_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_depth_target_decode(const mavlink_message_t* msg, mavlink_depth_target_t* depth_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    depth_target->byte1 = mavlink_msg_depth_target_get_byte1(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_depth_target_LEN? msg->len : MAVLINK_MSG_ID_depth_target_LEN;
        memset(depth_target, 0, MAVLINK_MSG_ID_depth_target_LEN);
    memcpy(depth_target, _MAV_PAYLOAD(msg), len);
#endif
}
