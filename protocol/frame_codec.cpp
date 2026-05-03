#include "frame_codec.hpp"
#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iostream>

namespace framecodec {

// --- CRC32 ---
static uint32_t crc32_ieee(const uint8_t* data, size_t len) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int j = 0; j < 8; ++j) c = (c >> 1) ^ (0xEDB88320u & (-(int)(c & 1)));
            table[i] = c;
        }
        init = true;
    }
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

// --- Small JSON helpers (schema-specific, not a general parser) ---
static std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return s;
}

static std::string extractString(const std::string& obj, const std::string& key) {
    const std::string pat = "\"" + key + "\"";
    auto pos = obj.find(pat);
    if (pos == std::string::npos) return {};
    pos = obj.find('"', pos + pat.size());
    if (pos == std::string::npos) return {};
    auto end = obj.find('"', pos + 1);
    if (end == std::string::npos) return {};
    return obj.substr(pos + 1, end - pos - 1);
}

static int extractInt(const std::string& obj, const std::string& key, int defval) {
    const std::string pat = "\"" + key + "\"";
    auto pos = obj.find(pat);
    if (pos == std::string::npos) return defval;
    pos = obj.find(':', pos + pat.size());
    if (pos == std::string::npos) return defval;
    while (pos < obj.size() && !std::isdigit(static_cast<unsigned char>(obj[pos])) && obj[pos] != '-') pos++;
    size_t end = pos;
    while (end < obj.size() && (std::isdigit(static_cast<unsigned char>(obj[end])) || obj[end] == '-')) end++;
    try { return std::stoi(obj.substr(pos, end - pos)); } catch (...) { return defval; }
}

static Schema parseSchema(const std::string& json) {
    Schema s;
    s.little_endian = (json.find("little") != std::string::npos);

    // locate fields array
    auto fldPos = json.find("\"fields\"");
    if (fldPos == std::string::npos) return s;
    auto lb = json.find('[', fldPos);
    auto rb = json.rfind(']');
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb) return s;
    std::string arr = json.substr(lb + 1, rb - lb - 1);

    size_t cur = 0;
    while (true) {
        auto b = arr.find('{', cur);
        if (b == std::string::npos) break;
        int depth = 0; size_t e = b;
        for (; e < arr.size(); ++e) {
            if (arr[e] == '{') depth++;
            else if (arr[e] == '}') { depth--; if (depth == 0) { e++; break; } }
        }
        if (e <= b) break;
        std::string obj = arr.substr(b, e - b);
        cur = e;

        Field f;
        f.name = extractString(obj, "name");
        f.type = extractString(obj, "type");
        f.count = extractInt(obj, "count", 1);
        f.size = extractInt(obj, "size", 0);
        if (!f.name.empty()) {
            if (f.type == "crc32" || f.name == "crc") {
                f.type = "uint32"; // normalize
            }
            s.fields.push_back(f);
            if (f.name == "crc") s.crc_index = static_cast<int>(s.fields.size() - 1);
        }
    }

    // compute frame size
    size_t total = 0;
    for (const auto& f: s.fields) {
        if (f.type == "uint8") total += 1;
        else if (f.type == "uint16") total += 2;
        else if (f.type == "uint32") total += 4;
        else if (f.type == "uint64") total += 8;
        else if (f.type == "float32") total += 4 * static_cast<size_t>(std::max(1, f.count));
        else if (f.type == "bytes" || f.type == "string") total += static_cast<size_t>(std::max(0, f.size));
    }
    s.frame_size = total;
    return s;
}

Schema loadSchema(const std::string& path) {
    std::string txt = readFile(path);
    if (txt.empty()) {
        std::cerr << "Failed to read schema file: " << path << "\n";
        return Schema{};
    }
    return parseSchema(txt);
}

const Schema& defaultSchema(const std::string& path) {
    static Schema cached = loadSchema(path);
    return cached;
}

// helper to write numbers
static void put_u8(std::vector<uint8_t>& buf, uint8_t v) { buf.push_back(v); }
static void put_u16(std::vector<uint8_t>& buf, uint16_t v, bool le) {
    if (le) { buf.push_back(v & 0xFF); buf.push_back((v>>8)&0xFF); }
    else { buf.push_back((v>>8)&0xFF); buf.push_back(v & 0xFF); }
}
static void put_u32(std::vector<uint8_t>& buf, uint32_t v, bool le) {
    if (le) { buf.push_back(v & 0xFF); buf.push_back((v>>8)&0xFF); buf.push_back((v>>16)&0xFF); buf.push_back((v>>24)&0xFF); }
    else { buf.push_back((v>>24)&0xFF); buf.push_back((v>>16)&0xFF); buf.push_back((v>>8)&0xFF); buf.push_back(v & 0xFF); }
}
static void put_u64(std::vector<uint8_t>& buf, uint64_t v, bool le) {
    uint32_t lo = static_cast<uint32_t>(v & 0xFFFFFFFFull);
    uint32_t hi = static_cast<uint32_t>((v >> 32) & 0xFFFFFFFFull);
    if (le) { put_u32(buf, lo, true); put_u32(buf, hi, true); }
    else { put_u32(buf, hi, false); put_u32(buf, lo, false); }
}
static void put_f32(std::vector<uint8_t>& buf, float f, bool le) {
    uint32_t raw; std::memcpy(&raw, &f, sizeof(float));
    put_u32(buf, raw, le);
}

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
                                const std::string& note) {
    std::vector<uint8_t> buf;
    buf.reserve(schema.frame_size ? schema.frame_size : 512);
    bool le = schema.little_endian;

    auto padFloats = [](const std::vector<float>& src, int count){
        std::vector<float> out(count, 0.0f);
        for (int i=0; i<count && i < static_cast<int>(src.size()); ++i) out[i] = src[i];
        return out;
    };
    auto padBytes = [](const std::vector<uint8_t>& src, int size){
        std::vector<uint8_t> out(size, 0);
        for (int i=0; i<size && i < static_cast<int>(src.size()); ++i) out[i] = src[i];
        return out;
    };

    int idx = 0;
    for (const auto& f : schema.fields) {
        // defer CRC: skip here, append after loop
        if (f.name == "crc" && f.type == "uint32") { idx++; continue; }
        if (f.type == "uint8") {
            uint8_t v = 0;
            if (f.name == "mode") v = mode;
            put_u8(buf, v);
        } else if (f.type == "uint16") {
            put_u16(buf, 0, le);
        } else if (f.type == "uint32") {
            uint32_t v = 0;
            if (f.name == "ind") v = ind;
            put_u32(buf, v, le);
        } else if (f.type == "uint64") {
            uint64_t v = (f.name == "time") ? time_ms : 0;
            put_u64(buf, v, le);
        } else if (f.type == "float32") {
            int count = std::max(1, f.count);
            std::vector<float> src;
            if (f.name == "angle") src = angles;
            else if (f.name == "current") src = currents;
            else if (f.name == "torque") src = torques;
            else if (f.name == "pose_ee") src = pose_ee;
            else if (f.name == "pose_elbow") src = pose_elbow;
            else if (f.name == "order") src = order;
            auto arr = padFloats(src, count);
            for (float x : arr) put_f32(buf, x, le);
        } else if (f.type == "bytes") {
            int size = std::max(0, f.size);
            std::vector<uint8_t> src = (f.name == "reserved") ? reserved : std::vector<uint8_t>{};
            auto arr = padBytes(src, size);
            buf.insert(buf.end(), arr.begin(), arr.end());
        } else if (f.type == "string") {
            int size = std::max(0, f.size);
            std::string s = (f.name == "note") ? note : std::string();
            if (static_cast<int>(s.size()) > size) s.resize(size);
            buf.insert(buf.end(), s.begin(), s.end());
            if (static_cast<int>(s.size()) < size) buf.insert(buf.end(), size - static_cast<int>(s.size()), 0);
        }
        idx++;
    }

    // append CRC if schema defines it (assumed at end)
    if (schema.crc_index >= 0) {
        uint32_t c = crc32_ieee(buf.data(), buf.size());
        put_u32(buf, c, le);
    }
    return buf;
}

std::vector<uint8_t> buildFrame(uint32_t ind, uint64_t time_ms,
                                const std::vector<float>& angles,
                                const std::vector<float>& currents,
                                const std::vector<float>& torques,
                                const std::vector<float>& pose_ee,
                                const std::vector<float>& pose_elbow,
                                uint8_t mode,
                                const std::vector<float>& order,
                                const std::vector<uint8_t>& reserved,
                                const std::string& note) {
    return buildFrame(defaultSchema(), ind, time_ms, angles, currents, torques, pose_ee, pose_elbow, mode, order, reserved, note);
}

} // namespace framecodec
