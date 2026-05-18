#pragma once
// MESSAGE CSDN_TEST PACKING

#define MAVLINK_MSG_ID_CSDN_TEST 236


typedef struct __mavlink_csdn_test_t {
 uint8_t byte1; /*<  FRIST BYTE.*/
 uint8_t byte2; /*<  SECOND BYTE.*/
} mavlink_csdn_test_t;

#define MAVLINK_MSG_ID_CSDN_TEST_LEN 2
#define MAVLINK_MSG_ID_CSDN_TEST_MIN_LEN 2
#define MAVLINK_MSG_ID_236_LEN 2
#define MAVLINK_MSG_ID_236_MIN_LEN 2

#define MAVLINK_MSG_ID_CSDN_TEST_CRC 239
#define MAVLINK_MSG_ID_236_CRC 239



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CSDN_TEST { \
    236, \
    "CSDN_TEST", \
    2, \
    {  { "byte1", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_csdn_test_t, byte1) }, \
         { "byte2", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_csdn_test_t, byte2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CSDN_TEST { \
    "CSDN_TEST", \
    2, \
    {  { "byte1", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_csdn_test_t, byte1) }, \
         { "byte2", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_csdn_test_t, byte2) }, \
         } \
}
#endif

/**
 * @brief Pack a csdn_test message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param byte1  FRIST BYTE.
 * @param byte2  SECOND BYTE.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_csdn_test_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t byte1, uint8_t byte2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CSDN_TEST_LEN];
    _mav_put_uint8_t(buf, 0, byte1);
    _mav_put_uint8_t(buf, 1, byte2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CSDN_TEST_LEN);
#else
    mavlink_csdn_test_t packet;
    packet.byte1 = byte1;
    packet.byte2 = byte2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CSDN_TEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CSDN_TEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CSDN_TEST_MIN_LEN, MAVLINK_MSG_ID_CSDN_TEST_LEN, MAVLINK_MSG_ID_CSDN_TEST_CRC);
}

/**
 * @brief Pack a csdn_test message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param byte1  FRIST BYTE.
 * @param byte2  SECOND BYTE.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_csdn_test_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t byte1,uint8_t byte2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CSDN_TEST_LEN];
    _mav_put_uint8_t(buf, 0, byte1);
    _mav_put_uint8_t(buf, 1, byte2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CSDN_TEST_LEN);
#else
    mavlink_csdn_test_t packet;
    packet.byte1 = byte1;
    packet.byte2 = byte2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CSDN_TEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CSDN_TEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CSDN_TEST_MIN_LEN, MAVLINK_MSG_ID_CSDN_TEST_LEN, MAVLINK_MSG_ID_CSDN_TEST_CRC);
}

/**
 * @brief Encode a csdn_test struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param csdn_test C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_csdn_test_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_csdn_test_t* csdn_test)
{
    return mavlink_msg_csdn_test_pack(system_id, component_id, msg, csdn_test->byte1, csdn_test->byte2);
}

/**
 * @brief Encode a csdn_test struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param csdn_test C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_csdn_test_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_csdn_test_t* csdn_test)
{
    return mavlink_msg_csdn_test_pack_chan(system_id, component_id, chan, msg, csdn_test->byte1, csdn_test->byte2);
}

/**
 * @brief Send a csdn_test message
 * @param chan MAVLink channel to send the message
 *
 * @param byte1  FRIST BYTE.
 * @param byte2  SECOND BYTE.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_csdn_test_send(mavlink_channel_t chan, uint8_t byte1, uint8_t byte2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CSDN_TEST_LEN];
    _mav_put_uint8_t(buf, 0, byte1);
    _mav_put_uint8_t(buf, 1, byte2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CSDN_TEST, buf, MAVLINK_MSG_ID_CSDN_TEST_MIN_LEN, MAVLINK_MSG_ID_CSDN_TEST_LEN, MAVLINK_MSG_ID_CSDN_TEST_CRC);
#else
    mavlink_csdn_test_t packet;
    packet.byte1 = byte1;
    packet.byte2 = byte2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CSDN_TEST, (const char *)&packet, MAVLINK_MSG_ID_CSDN_TEST_MIN_LEN, MAVLINK_MSG_ID_CSDN_TEST_LEN, MAVLINK_MSG_ID_CSDN_TEST_CRC);
#endif
}

/**
 * @brief Send a csdn_test message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_csdn_test_send_struct(mavlink_channel_t chan, const mavlink_csdn_test_t* csdn_test)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_csdn_test_send(chan, csdn_test->byte1, csdn_test->byte2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CSDN_TEST, (const char *)csdn_test, MAVLINK_MSG_ID_CSDN_TEST_MIN_LEN, MAVLINK_MSG_ID_CSDN_TEST_LEN, MAVLINK_MSG_ID_CSDN_TEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_CSDN_TEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_csdn_test_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t byte1, uint8_t byte2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, byte1);
    _mav_put_uint8_t(buf, 1, byte2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CSDN_TEST, buf, MAVLINK_MSG_ID_CSDN_TEST_MIN_LEN, MAVLINK_MSG_ID_CSDN_TEST_LEN, MAVLINK_MSG_ID_CSDN_TEST_CRC);
#else
    mavlink_csdn_test_t *packet = (mavlink_csdn_test_t *)msgbuf;
    packet->byte1 = byte1;
    packet->byte2 = byte2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CSDN_TEST, (const char *)packet, MAVLINK_MSG_ID_CSDN_TEST_MIN_LEN, MAVLINK_MSG_ID_CSDN_TEST_LEN, MAVLINK_MSG_ID_CSDN_TEST_CRC);
#endif
}
#endif

#endif

// MESSAGE CSDN_TEST UNPACKING


/**
 * @brief Get field byte1 from csdn_test message
 *
 * @return  FRIST BYTE.
 */
static inline uint8_t mavlink_msg_csdn_test_get_byte1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field byte2 from csdn_test message
 *
 * @return  SECOND BYTE.
 */
static inline uint8_t mavlink_msg_csdn_test_get_byte2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a csdn_test message into a struct
 *
 * @param msg The message to decode
 * @param csdn_test C-struct to decode the message contents into
 */
static inline void mavlink_msg_csdn_test_decode(const mavlink_message_t* msg, mavlink_csdn_test_t* csdn_test)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    csdn_test->byte1 = mavlink_msg_csdn_test_get_byte1(msg);
    csdn_test->byte2 = mavlink_msg_csdn_test_get_byte2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CSDN_TEST_LEN? msg->len : MAVLINK_MSG_ID_CSDN_TEST_LEN;
        memset(csdn_test, 0, MAVLINK_MSG_ID_CSDN_TEST_LEN);
    memcpy(csdn_test, _MAV_PAYLOAD(msg), len);
#endif
}
