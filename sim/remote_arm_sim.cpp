#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

// UDP frame publisher at 100 Hz following sim/frame_config.json
#include "frame_codec.hpp"
using framecodec::buildFrame;
using framecodec::defaultSchema;

struct SocketHandle {
#if defined(_WIN32)
    SOCKET fd = INVALID_SOCKET;
    ~SocketHandle() { if (fd != INVALID_SOCKET) closesocket(fd); }
#else
    int fd = -1;
    ~SocketHandle() { if (fd >= 0) close(fd); }
#endif
};

int main(int argc, char** argv) {
    const char* defaultHost = "127.0.0.1";
    int defaultPort = 14550;
    double defaultHz = 100.0;

    const char* host = (argc > 1) ? argv[1] : defaultHost;
    int port = (argc > 2) ? std::atoi(argv[2]) : defaultPort;
    double hz = (argc > 3) ? std::atof(argv[3]) : defaultHz;
    if (hz <= 0.0) hz = defaultHz;

#if defined(_WIN32)
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed" << '\n';
        return 1;
    }
#endif

    SocketHandle sock;
    sock.fd = static_cast<decltype(sock.fd)>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (
#if defined(_WIN32)
        sock.fd == INVALID_SOCKET
#else
        sock.fd < 0
#endif
    ) {
        std::cerr << "Failed to create UDP socket" << '\n';
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        std::cerr << "Invalid host: " << host << '\n';
        return 1;
    }

    const auto& schema = defaultSchema();
    auto findCount = [&](const std::string& name){
        for (const auto& f : schema.fields) if (f.name == name) return (f.type == "float32") ? std::max(1, f.count) : 0; return 0; };
    auto findSize = [&](const std::string& name){
        for (const auto& f : schema.fields) if (f.name == name) return (f.type == "bytes" || f.type == "string") ? std::max(0, f.size) : 0; return 0; };

    const int angleN = findCount("angle");
    const int currentN = findCount("current");
    const int torqueN = findCount("torque");
    const int poseN = findCount("pose_ee");
    const int poseElbowN = findCount("pose_elbow");
    const int orderN = findCount("order");
    const int reservedSize = findSize("reserved");

    std::cout << "Sending schema UDP frames to " << host << ':' << port
              << " at " << hz << " Hz" << " (angleN=" << angleN << ")" << '\n';

    auto period = std::chrono::duration<double>(1.0 / hz);
    auto next = std::chrono::steady_clock::now();
    auto start = next;
    uint32_t ind = 0;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - start).count();

        std::vector<float> angles(angleN, 0.0f);
        std::vector<float> currents(currentN, 0.0f);
        for (int i = 0; i < angleN; ++i) {
            angles[i] = static_cast<float>(30.0 * std::sin(t + i * 0.3) + i * 5.0);
        }
        for (int i = 0; i < currentN; ++i) {
            currents[i] = static_cast<float>(2.0 + 0.5 * std::cos(t + i * 0.4));
        }

        const uint64_t tsMs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
        std::vector<float> torque = currents; // simple placeholder relation
        std::vector<float> poseEE(poseN, 0.0f), poseElbow(poseElbowN, 0.0f);
        for (int i=0;i<poseN && i<static_cast<int>(angles.size()); ++i){ poseEE[i]=angles[i]*0.01f; }
        for (int i=0;i<poseElbowN && i<static_cast<int>(angles.size()); ++i){ poseElbow[i]=angles[i]*0.008f; }
        // sub-endpoint semantics: set non-valid fields to invalid fill
        std::vector<float> order(orderN, -1.0f);
        std::vector<uint8_t> reserved(static_cast<size_t>(reservedSize), 255);
        // mode: status nibble 'ready' (0001), low nibble mirror (none=0000)
        uint8_t mode = static_cast<uint8_t>((1 << 4) | 0);
        auto frame = buildFrame(schema, ind++, tsMs, angles, currents, torque, poseEE, poseElbow, mode, order, reserved, "sim");

        const int sent = sendto(
            sock.fd,
            reinterpret_cast<const char*>(frame.data()),
            static_cast<int>(frame.size()),
            0,
            reinterpret_cast<sockaddr*>(&addr),
            static_cast<int>(sizeof(addr))
        );
        if (sent < 0) {
            std::cerr << "sendto failed" << '\n';
        }

        // light console debug at ~2 Hz
        static int tick = 0;
        if ((tick++ % static_cast<int>(hz / 2)) == 0) {
            std::cout << "tx frame ts=" << tsMs
                      << " a0=" << (angles.empty()?0:angles[0])
                      << " c0=" << (currents.empty()?0:currents[0])
                      << " -> " << host << ':' << port
                      << " bytes=" << frame.size()
                      << '\n';
        }

        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
    }

#if defined(_WIN32)
    WSACleanup();
#endif
    return 0;
}
