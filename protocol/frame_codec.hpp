#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace framecodec {

struct Field {
    std::string name;
    std::string type;
    int count = 1;   // for float32 arrays
    int size = 0;    // for bytes/string
};

struct Schema {
    bool little_endian = true;
    std::vector<Field> fields;
    int crc_index = -1; // index in fields if present
    size_t frame_size = 0;
};

// Load schema from JSON file. Minimal tolerant parser tailored for frame_config.json structure.
Schema loadSchema(const std::string& path);

// Cached default schema (path defaults to "sim/frame_config.json").
const Schema& defaultSchema(const std::string& path = "sim/frame_config.json");

// Build a frame using the provided schema. Arrays/vectors are padded/truncated to schema counts.
std::vector<uint8_t> buildFrame(const Schema& schema,
                                uint32_t ind, uint64_t time_ms,
                                const std::vector<float>& angles,
                                const std::vector<float>& currents,
                                const std::vector<float>& torques,
                                const std::vector<float>& pose_ee,
                                const std::vector<float>& pose_elbow,
                                uint8_t mode,
                                const std::vector<float>& order,
                                const std::vector<uint8_t>& reserved,
                                const std::string& note);

// Convenience wrapper using default schema path.
std::vector<uint8_t> buildFrame(uint32_t ind, uint64_t time_ms,
                                const std::vector<float>& angles,
                                const std::vector<float>& currents,
                                const std::vector<float>& torques,
                                const std::vector<float>& pose_ee,
                                const std::vector<float>& pose_elbow,
                                uint8_t mode,
                                const std::vector<float>& order,
                                const std::vector<uint8_t>& reserved,
                                const std::string& note);

}
