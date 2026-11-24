#include "serial_port.h"
#include <mavlink/fcu_messages.h>
#include <mavlink/mavlink_codec.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <thread>

using ego_mav::MavMessage;
using ego_mav::Parser;
using ego_onboard::SerialPort;

namespace {
void usage() {
  std::puts("mavlink_rx_test --serial <device> [--baud <baud>] [--duration <seconds>]");
  std::puts("Receive-only MAVLink v1. Default baud=921600 duration=10s.");
}
}

int main(int argc, char **argv) {
  std::string device;
  int baud = 921600;
  double duration = 10.0;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      auto value = [&]() -> std::string {
        if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
        return argv[++i];
      };
      if (arg == "--serial") device = value();
      else if (arg == "--baud") baud = std::stoi(value());
      else if (arg == "--duration") duration = std::stod(value());
      else if (arg == "--help" || arg == "-h") { usage(); return 0; }
      else throw std::runtime_error("unknown option: " + arg);
    }
    if (device.empty()) throw std::runtime_error("--serial is required");
    if (!std::isfinite(duration) || duration <= 0 || duration > 3600)
      throw std::runtime_error("duration must be in (0, 3600]");

    SerialPort serial;
    std::string error;
    if (!serial.open(device, baud, error)) throw std::runtime_error(error);

    Parser parser;
    MavMessage message;
    std::map<uint32_t, uint64_t> message_counts;
    std::set<std::pair<uint8_t, uint8_t>> endpoints;
    uint64_t bytes = 0, checked_frames = 0, unchecked_frames = 0, heartbeats = 0;
    const auto begin = std::chrono::steady_clock::now();
    auto next_report = begin;
    std::puts("[RX ONLY] serial opened; no MAVLink data will be transmitted");
    std::printf("[CONFIG] device=%s baud=%d duration=%.1fs\n", device.c_str(), baud, duration);

    while (std::chrono::steady_clock::now() - begin < std::chrono::duration<double>(duration)) {
      if (!serial.isOpen()) throw std::runtime_error("serial reader disconnected");
      std::vector<uint8_t> chunk;
      serial.take(chunk);
      bytes += chunk.size();
      for (uint8_t byte : chunk) {
        if (!parser.push(byte, message)) continue;
        ++message_counts[message.msgid];
        endpoints.insert({message.sysid, message.compid});
        if (message.crc_checked) ++checked_frames;
        else ++unchecked_frames;
        if (message.msgid == ego_mav::Heartbeat::ID && message.crc_checked) {
          ego_mav::Heartbeat heartbeat;
          if (ego_mav::Heartbeat::decode(message.payload.data(), message.payload.size(), heartbeat)) {
            ++heartbeats;
            std::printf("[HEARTBEAT] sysid=%u compid=%u type=%u autopilot=%u base_mode=0x%02x armed=%d main_mode=%u sub_mode=%u status=%u\n",
                        message.sysid, message.compid, heartbeat.type, heartbeat.autopilot,
                        heartbeat.base_mode, heartbeat.armed(), heartbeat.mainMode(), heartbeat.subMode(),
                        heartbeat.system_status);
          }
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (now >= next_report) {
        next_report = now + std::chrono::seconds(1);
        std::printf("[RX] bytes=%llu valid_crc_frames=%llu unchecked_frames=%llu crc_errors=%llu heartbeats=%llu\n",
                    static_cast<unsigned long long>(bytes),
                    static_cast<unsigned long long>(checked_frames),
                    static_cast<unsigned long long>(unchecked_frames),
                    static_cast<unsigned long long>(parser.crcErrors()),
                    static_cast<unsigned long long>(heartbeats));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    serial.close();

    std::puts("[SUMMARY] received MAVLink endpoints:");
    for (const auto &endpoint : endpoints)
      std::printf("  sysid=%u compid=%u\n", endpoint.first, endpoint.second);
    std::puts("[SUMMARY] message counts (msgid: count):");
    for (const auto &entry : message_counts)
      std::printf("  %u: %llu\n", entry.first, static_cast<unsigned long long>(entry.second));
    std::printf("[RESULT] bytes=%llu valid_crc_frames=%llu unchecked_frames=%llu crc_errors=%llu heartbeats=%llu\n",
                static_cast<unsigned long long>(bytes),
                static_cast<unsigned long long>(checked_frames),
                static_cast<unsigned long long>(unchecked_frames),
                static_cast<unsigned long long>(parser.crcErrors()),
                static_cast<unsigned long long>(heartbeats));
    return checked_frames > 0 ? 0 : 1;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "[ERROR] %s\n", error.what());
    usage();
    return 2;
  }
}
