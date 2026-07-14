#include "mainpulator/online_joint_planner.h"

#include <algorithm>
#include <cmath>

namespace mainpulator_motion {

OnlineJointPlanner::OnlineJointPlanner(double control_period_sec)
    : otg_(control_period_sec) {
    input_.control_interface = ruckig::ControlInterface::Position;
    input_.synchronization = ruckig::Synchronization::Time;
    input_.duration_discretization = ruckig::DurationDiscretization::Discrete;
}

bool OnlineJointPlanner::allFinite(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

bool OnlineJointPlanner::allPositiveFinite(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value) && value > 0.0;
    });
}

bool OnlineJointPlanner::configure(const JointVector& max_velocity,
                                   const JointVector& max_acceleration,
                                   const JointVector& max_jerk,
                                   std::string* error) {
    if (!allPositiveFinite(max_velocity) || !allPositiveFinite(max_acceleration) ||
        !allPositiveFinite(max_jerk)) {
        if (error != nullptr) {
            *error = "trajectory limits must contain four positive finite values";
        }
        configured_ = false;
        return false;
    }

    input_.max_velocity = max_velocity;
    input_.max_acceleration = max_acceleration;
    input_.max_jerk = max_jerk;
    configured_ = true;
    return true;
}

bool OnlineJointPlanner::initialize(const JointVector& position,
                                    const JointVector& velocity,
                                    const JointVector& acceleration,
                                    std::string* error) {
    if (!configured_) {
        if (error != nullptr) {
            *error = "trajectory planner is not configured";
        }
        return false;
    }
    if (!allFinite(position) || !allFinite(velocity) || !allFinite(acceleration)) {
        if (error != nullptr) {
            *error = "initial trajectory state contains a non-finite value";
        }
        return false;
    }

    input_.current_position = position;
    input_.current_velocity = velocity;
    input_.current_acceleration = acceleration;
    input_.target_position = position;
    input_.target_velocity = JointVector {{0.0, 0.0, 0.0, 0.0}};
    input_.target_acceleration = JointVector {{0.0, 0.0, 0.0, 0.0}};
    position_ = position;
    velocity_ = velocity;
    acceleration_ = acceleration;
    jerk_.fill(0.0);
    otg_.reset();
    output_.time = 0.0;
    initialized_ = true;
    last_result_ = ruckig::Result::Finished;
    return true;
}

bool OnlineJointPlanner::setTarget(const JointVector& position,
                                   const JointVector& velocity,
                                   const JointVector& acceleration,
                                   std::string* error) {
    if (!initialized_) {
        if (error != nullptr) {
            *error = "trajectory planner is not initialized";
        }
        return false;
    }
    if (!allFinite(position) || !allFinite(velocity) || !allFinite(acceleration)) {
        if (error != nullptr) {
            *error = "trajectory target contains a non-finite value";
        }
        return false;
    }

    input_.target_position = position;
    input_.target_velocity = velocity;
    input_.target_acceleration = acceleration;
    return true;
}

ruckig::Result OnlineJointPlanner::update() {
    if (!initialized_) {
        last_result_ = ruckig::Result::ErrorInvalidInput;
        return last_result_;
    }

    last_result_ = otg_.update(input_, output_);
    if (last_result_ == ruckig::Result::Working || last_result_ == ruckig::Result::Finished) {
        output_.pass_to_input(input_);
        copyOutputState();
    }
    return last_result_;
}

void OnlineJointPlanner::hold() {
    if (!initialized_) {
        return;
    }
    velocity_.fill(0.0);
    acceleration_.fill(0.0);
    jerk_.fill(0.0);
    input_.current_position = position_;
    input_.current_velocity = velocity_;
    input_.current_acceleration = acceleration_;
    input_.target_position = position_;
    input_.target_velocity = velocity_;
    input_.target_acceleration = acceleration_;
    otg_.reset();
    output_.time = 0.0;
    last_result_ = ruckig::Result::Finished;
}

void OnlineJointPlanner::copyOutputState() {
    position_ = output_.new_position;
    velocity_ = output_.new_velocity;
    acceleration_ = output_.new_acceleration;
    jerk_ = output_.new_jerk;
}

double OnlineJointPlanner::trajectoryDuration() const {
    return initialized_ ? output_.trajectory.get_duration() : 0.0;
}

double OnlineJointPlanner::progress() const {
    const double duration = trajectoryDuration();
    if (duration <= 0.0) {
        return last_result_ == ruckig::Result::Finished ? 1.0 : 0.0;
    }
    return std::max(0.0, std::min(1.0, output_.time / duration));
}

const char* OnlineJointPlanner::resultName(ruckig::Result result) {
    switch (result) {
        case ruckig::Result::Working: return "working";
        case ruckig::Result::Finished: return "finished";
        case ruckig::Result::ErrorInvalidInput: return "error_invalid_input";
        case ruckig::Result::ErrorTrajectoryDuration: return "error_trajectory_duration";
        case ruckig::Result::ErrorPositionalLimits: return "error_positional_limits";
        case ruckig::Result::ErrorZeroLimits: return "error_zero_limits";
        case ruckig::Result::ErrorExecutionTimeCalculation: return "error_execution_time";
        case ruckig::Result::ErrorSynchronizationCalculation: return "error_synchronization";
        default: return "error";
    }
}

}  // namespace mainpulator_motion
