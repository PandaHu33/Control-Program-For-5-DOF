#include <gtest/gtest.h>

#include <can_msgs/Frame.h>
#include <linux/can.h>

#include "mainpulator/mainpulator_control.h"

TEST(ActualCurrent, ConvertsSignedPerMilleToMilliamps)
{
    control::mainpulator joint(1, 0);
    struct can_frame rated = {};
    rated.can_dlc = 8;
    rated.data[4] = 0xD0;  // 2000 mA, little endian.
    rated.data[5] = 0x07;
    joint.Electric_init(rated);

    can_msgs::Frame positive;
    positive.dlc = 2;
    positive.data[0] = 0xF4;  // +500 per mille.
    positive.data[1] = 0x01;
    joint.ActualCurrent(positive);
    EXPECT_DOUBLE_EQ(joint.get_ActualCurrent(), 1000.0);

    can_msgs::Frame negative;
    negative.dlc = 2;
    negative.data[0] = 0x18;  // -1000 per mille (0xFC18).
    negative.data[1] = 0xFC;
    joint.ActualCurrent(negative);
    EXPECT_DOUBLE_EQ(joint.get_ActualCurrent(), -2000.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
