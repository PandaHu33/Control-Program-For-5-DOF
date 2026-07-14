#pragma once

#include <array>
#include <cstddef>
#include <string>

#include <ruckig/ruckig.hpp>

namespace mainpulator_motion {

constexpr std::size_t kArmPlannerDofs = 4;
using JointVector = std::array<double, kArmPlannerDofs>;

class OnlineJointPlanner {
public:
    explicit OnlineJointPlanner(double control_period_sec = 0.01);

    bool configure(const JointVector& max_velocity,
                   const JointVector& max_acceleration,
                   const JointVector& max_jerk,
                   std::string* error = nullptr);
    bool initialize(const JointVector& position,
                    const JointVector& velocity,
                    const JointVector& acceleration,
                    std::string* error = nullptr);
    bool setTarget(const JointVector& position,
                   const JointVector& velocity,
                   const JointVector& acceleration,
                   std::string* error = nullptr);
    ruckig::Result update();
    void hold();

    bool initialized() const { return initialized_; }
    ruckig::Result result() const { return last_result_; }
    const JointVector& position() const { return position_; }
    const JointVector& velocity() const { return velocity_; }
    const JointVector& acceleration() const { return acceleration_; }
    const JointVector& jerk() const { return jerk_; }
    double trajectoryDuration() const;
    double trajectoryTime() const { return output_.time; }
    double progress() const;
    static const char* resultName(ruckig::Result result);

private:
    static bool allFinite(const JointVector& values);
    static bool allPositiveFinite(const JointVector& values);
    void copyOutputState();

    ruckig::Ruckig<kArmPlannerDofs> otg_;
    ruckig::InputParameter<kArmPlannerDofs> input_;
    ruckig::OutputParameter<kArmPlannerDofs> output_;
    JointVector position_ {{0.0, 0.0, 0.0, 0.0}};
    JointVector velocity_ {{0.0, 0.0, 0.0, 0.0}};
    JointVector acceleration_ {{0.0, 0.0, 0.0, 0.0}};
    JointVector jerk_ {{0.0, 0.0, 0.0, 0.0}};
    bool configured_ {false};
    bool initialized_ {false};
    ruckig::Result last_result_ {ruckig::Result::Finished};
};

}  // namespace mainpulator_motion
