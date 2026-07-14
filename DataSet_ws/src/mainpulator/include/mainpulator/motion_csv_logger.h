#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace mainpulator_motion {

struct MotionLogSample {
    std::uint64_t wall_time_ns {0};
    std::uint64_t ros_time_ns {0};
    std::uint64_t loop_index {0};
    double dt_sec {0.0};
    std::string active_source;
    std::uint32_t command_ind {0};
    bool planner_enabled {false};
    std::string planner_result;
    std::array<double, 4> raw_q {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> raw_dq {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> raw_ddq {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> expected_q {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> expected_dq {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> expected_ddq {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> actual_q {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> actual_dq {{0.0, 0.0, 0.0, 0.0}};
};

struct MotionCsvLoggerConfig {
    std::string path;
    std::size_t queue_capacity {8192};
    std::size_t flush_rows {100};
};

class MotionCsvLogger {
public:
    MotionCsvLogger() = default;
    ~MotionCsvLogger();

    MotionCsvLogger(const MotionCsvLogger&) = delete;
    MotionCsvLogger& operator=(const MotionCsvLogger&) = delete;

    bool start(const MotionCsvLoggerConfig& config, std::string* error = nullptr);
    bool push(const MotionLogSample& sample);
    void stop();

    bool running() const { return running_.load(std::memory_order_acquire); }
    std::uint64_t droppedSamples() const { return dropped_samples_.load(std::memory_order_relaxed); }
    const std::string& sessionId() const { return session_id_; }

private:
    static bool ensureParentDirectory(const std::string& path);
    static std::string csvCell(const std::string& value);
    static void writeHeader(std::ostream& stream);
    void writeSample(const MotionLogSample& sample);
    void writerLoop();

    MotionCsvLoggerConfig config_;
    std::string session_id_;
    std::ofstream file_;
    std::deque<MotionLogSample> queue_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::thread writer_thread_;
    std::atomic<bool> running_ {false};
    std::atomic<bool> stop_requested_ {false};
    std::atomic<std::uint64_t> dropped_samples_ {0};
};

}  // namespace mainpulator_motion
