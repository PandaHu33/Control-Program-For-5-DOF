#include "mainpulator/motion_csv_logger.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <initializer_list>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace mainpulator_motion {
namespace {

template <typename Vector>
void writeVector(std::ostream& stream, const Vector& values) {
    for (double value : values) {
        stream << ',' << value;
    }
}

}  // namespace

MotionCsvLogger::~MotionCsvLogger() {
    stop();
}

bool MotionCsvLogger::ensureParentDirectory(const std::string& path) {
    const std::size_t separator = path.find_last_of("/\\");
    if (separator == std::string::npos) {
        return true;
    }
    const std::string parent = path.substr(0, separator);
    if (parent.empty()) {
        return true;
    }

    std::string current;
    if (parent.front() == '/') {
        current = "/";
    }
    std::size_t start = parent.front() == '/' ? 1 : 0;
    while (start <= parent.size()) {
        const std::size_t end = parent.find_first_of("/\\", start);
        const std::string part = parent.substr(start, end - start);
        if (!part.empty()) {
            if (!current.empty() && current.back() != '/') {
                current.push_back('/');
            }
            current += part;
            if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
                return false;
            }
        }
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return true;
}

bool MotionCsvLogger::start(const MotionCsvLoggerConfig& config, std::string* error) {
    if (running()) {
        if (error != nullptr) {
            *error = "motion CSV logger is already running";
        }
        return false;
    }
    if (config.path.empty() || config.queue_capacity == 0 || config.flush_rows == 0) {
        if (error != nullptr) {
            *error = "motion CSV logger path, queue capacity, and flush rows must be non-zero";
        }
        return false;
    }
    if (!ensureParentDirectory(config.path)) {
        if (error != nullptr) {
            *error = "failed to create motion log parent directory: " + std::string(std::strerror(errno));
        }
        return false;
    }

    std::ifstream existing(config.path.c_str(), std::ios::binary);
    const bool empty = !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    file_.open(config.path.c_str(), std::ios::out | std::ios::app);
    if (!file_.good()) {
        if (error != nullptr) {
            *error = "failed to open motion CSV log: " + config.path;
        }
        return false;
    }

    config_ = config;
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    session_id_ = std::to_string(now_ns);
    dropped_samples_.store(0, std::memory_order_relaxed);
    stop_requested_.store(false, std::memory_order_release);
    if (empty) {
        writeHeader(file_);
        file_.flush();
    }
    running_.store(true, std::memory_order_release);
    writer_thread_ = std::thread(&MotionCsvLogger::writerLoop, this);
    return true;
}

bool MotionCsvLogger::push(const MotionLogSample& sample) {
    if (!running()) {
        return false;
    }
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || queue_.size() >= config_.queue_capacity) {
        dropped_samples_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    queue_.push_back(sample);
    lock.unlock();
    condition_.notify_one();
    return true;
}

void MotionCsvLogger::stop() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    stop_requested_.store(true, std::memory_order_release);
    condition_.notify_all();
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    if (file_.good()) {
        file_.flush();
        file_.close();
    }
}

std::string MotionCsvLogger::csvCell(const std::string& value) {
    if (value.find_first_of(",\"\r\n") == std::string::npos) {
        return value;
    }
    std::string escaped = "\"";
    for (char ch : value) {
        if (ch == '"') {
            escaped += "\"\"";
        } else {
            escaped.push_back(ch);
        }
    }
    escaped.push_back('"');
    return escaped;
}

void MotionCsvLogger::writeHeader(std::ostream& stream) {
    stream << "session_id,wall_time_ns,ros_time_ns,loop_index,dt_sec,active_source,command_ind,planner_enabled,planner_result";
    for (const char* prefix : {"raw_q", "raw_dq", "raw_ddq", "expected_q", "expected_dq", "expected_ddq", "actual_q", "actual_dq"}) {
        for (std::size_t joint = 1; joint <= 4; ++joint) {
            stream << ',' << prefix << joint;
        }
    }
    stream << ",dropped_log_samples\n";
}

void MotionCsvLogger::writeSample(const MotionLogSample& sample) {
    file_ << session_id_ << ',' << sample.wall_time_ns << ',' << sample.ros_time_ns << ','
          << sample.loop_index << ',' << std::setprecision(17) << sample.dt_sec << ','
          << csvCell(sample.active_source) << ',' << sample.command_ind << ','
          << (sample.planner_enabled ? 1 : 0) << ',' << csvCell(sample.planner_result);
    writeVector(file_, sample.raw_q);
    writeVector(file_, sample.raw_dq);
    writeVector(file_, sample.raw_ddq);
    writeVector(file_, sample.expected_q);
    writeVector(file_, sample.expected_dq);
    writeVector(file_, sample.expected_ddq);
    writeVector(file_, sample.actual_q);
    writeVector(file_, sample.actual_dq);
    file_ << ',' << droppedSamples() << '\n';
}

void MotionCsvLogger::writerLoop() {
    std::size_t rows_since_flush = 0;
    auto last_flush = std::chrono::steady_clock::now();
    while (true) {
        std::deque<MotionLogSample> batch;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait_for(lock, std::chrono::seconds(1), [this] {
                return stop_requested_.load(std::memory_order_acquire) || !queue_.empty();
            });
            batch.swap(queue_);
            if (batch.empty() && stop_requested_.load(std::memory_order_acquire)) {
                break;
            }
        }

        for (const MotionLogSample& sample : batch) {
            writeSample(sample);
            ++rows_since_flush;
        }
        const auto now = std::chrono::steady_clock::now();
        if (rows_since_flush >= config_.flush_rows || now - last_flush >= std::chrono::seconds(1)) {
            file_.flush();
            rows_since_flush = 0;
            last_flush = now;
        }
    }
    file_.flush();
}

}  // namespace mainpulator_motion
