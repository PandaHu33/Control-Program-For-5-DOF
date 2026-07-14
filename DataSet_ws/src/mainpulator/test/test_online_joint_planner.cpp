#include "mainpulator/online_joint_planner.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

namespace {

using mainpulator_motion::JointVector;
using mainpulator_motion::OnlineJointPlanner;

TEST(OnlineJointPlanner, StepReverseAndRepeatedTargetsStayWithinLimits) {
    const JointVector max_velocity {{0.5, 0.5, 0.5, 0.5}};
    const JointVector max_acceleration {{1.0, 1.0, 1.0, 1.0}};
    const JointVector max_jerk {{5.0, 5.0, 5.0, 5.0}};
    const JointVector zero {{0.0, 0.0, 0.0, 0.0}};

    OnlineJointPlanner planner(0.01);
    ASSERT_TRUE(planner.configure(max_velocity, max_acceleration, max_jerk));
    ASSERT_TRUE(planner.initialize(zero, zero, zero));
    ASSERT_TRUE(planner.setTarget(JointVector {{1.0, -0.8, 0.6, -0.4}}, zero, zero));

    JointVector previous_q = planner.position();
    for (int cycle = 0; cycle < 800; ++cycle) {
        if (cycle == 100) {
            ASSERT_TRUE(planner.setTarget(JointVector {{-0.7, 0.5, -0.3, 0.9}}, zero, zero));
        }
        const ruckig::Result result = planner.update();
        ASSERT_TRUE(result == ruckig::Result::Working || result == ruckig::Result::Finished);
        for (std::size_t joint = 0; joint < 4; ++joint) {
            EXPECT_LE(std::abs(planner.position()[joint] - previous_q[joint]),
                      max_velocity[joint] * 0.01 + 1e-9);
            EXPECT_LE(std::abs(planner.velocity()[joint]), max_velocity[joint] + 1e-9);
            EXPECT_LE(std::abs(planner.acceleration()[joint]), max_acceleration[joint] + 1e-9);
            EXPECT_LE(std::abs(planner.jerk()[joint]), max_jerk[joint] + 1e-9);
        }
        previous_q = planner.position();
    }
}

TEST(OnlineJointPlanner, TenCycleTargetUpdatesRemainFiniteAndBounded) {
    const JointVector max_velocity {{0.5, 0.5, 0.5, 0.5}};
    const JointVector max_acceleration {{1.0, 1.0, 1.0, 1.0}};
    const JointVector max_jerk {{5.0, 5.0, 5.0, 5.0}};
    const JointVector zero {{0.0, 0.0, 0.0, 0.0}};
    OnlineJointPlanner planner(0.01);
    ASSERT_TRUE(planner.configure(max_velocity, max_acceleration, max_jerk));
    ASSERT_TRUE(planner.initialize(zero, zero, zero));

    std::mt19937 generator(42);
    std::uniform_real_distribution<double> target_distribution(-1.0, 1.0);
    for (int cycle = 0; cycle < 1000; ++cycle) {
        if (cycle % 10 == 0) {
            JointVector target;
            for (double& value : target) value = target_distribution(generator);
            ASSERT_TRUE(planner.setTarget(target, zero, zero));
        }
        const ruckig::Result result = planner.update();
        ASSERT_TRUE(result == ruckig::Result::Working || result == ruckig::Result::Finished);
        for (std::size_t joint = 0; joint < 4; ++joint) {
            EXPECT_TRUE(std::isfinite(planner.position()[joint]));
            EXPECT_LE(std::abs(planner.velocity()[joint]), max_velocity[joint] + 1e-9);
            EXPECT_LE(std::abs(planner.acceleration()[joint]), max_acceleration[joint] + 1e-9);
            EXPECT_LE(std::abs(planner.jerk()[joint]), max_jerk[joint] + 1e-9);
        }
    }
}

TEST(OnlineJointPlanner, RejectsNonFiniteTargetWithoutChangingCurrentState) {
    const JointVector limits {{1.0, 1.0, 1.0, 1.0}};
    const JointVector zero {{0.0, 0.0, 0.0, 0.0}};
    OnlineJointPlanner planner(0.01);
    ASSERT_TRUE(planner.configure(limits, limits, limits));
    ASSERT_TRUE(planner.initialize(zero, zero, zero));
    JointVector invalid = zero;
    invalid[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(planner.setTarget(invalid, zero, zero));
    EXPECT_EQ(planner.update(), ruckig::Result::Finished);
    EXPECT_EQ(planner.position(), zero);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
