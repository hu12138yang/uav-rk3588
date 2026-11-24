#include "serial_port.h"
#include <mavlink/fcu_messages.h>
#include <mavlink/mavlink_codec.h>
#include <chrono>
#include <cstdio>
#include <deque>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using ego_mav::MavMessage;
using ego_mav::Parser;
using ego_onboard::SerialPort;

namespace {
void usage() {
  std::puts("mavlink_offboard_once --serial <device> [--baud <baud>] [--mode-only] [--v1]");
  std::puts("Default: arm once, wait for confirmation, then request OFFBOARD once.");
  std::puts("--mode-only: send one OFFBOARD request and, after confirmation, send static local-NED setpoints.");
  std::puts("Does not take off or send position setpoints.");
}

bool takeMessage(SerialPort &serial, Parser &parser, MavMessage &message,
                 std::deque<MavMessage> &pending,
                 std::chrono::steady_clock::time_point deadline) {
  while (std::chrono::steady_clock::now() < deadline) {
    if (!pending.empty()) {
      message = std::move(pending.front());
      pending.pop_front();
      return true;
    }
    if (!serial.isOpen()) throw std::runtime_error("serial reader disconnected");
    std::vector<uint8_t> bytes;
    serial.take(bytes);
    for (uint8_t byte : bytes) {
      MavMessage parsed;
      if (parser.push(byte, parsed)) pending.push_back(std::move(parsed));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}
}

int main(int argc, char **argv) {
  std::string device;
  int baud = 921600;
  bool mode_only = false;
  bool mavlink_v1 = true;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--serial" && i + 1 < argc) device = argv[++i];
      else if (arg == "--baud" && i + 1 < argc) baud = std::stoi(argv[++i]);
      else if (arg == "--mode-only") mode_only = true;
      else if (arg == "--v1") mavlink_v1 = true;
      else if (arg == "--v2") mavlink_v1 = false;
      else if (arg == "--help" || arg == "-h") { usage(); return 0; }
      else throw std::runtime_error("invalid or incomplete option: " + arg);
    }
    if (device.empty()) throw std::runtime_error("--serial is required");

    constexpr uint8_t source_system = 245;
    constexpr uint8_t source_component = 1;
    constexpr uint8_t target_system = 1;
    constexpr uint8_t target_component = 1;

    SerialPort serial;
    std::string error;
    if (!serial.open(device, baud, error)) throw std::runtime_error(error);
    Parser parser;
    MavMessage message;
    std::deque<MavMessage> pending;
    bool ready = false;
    const auto observe_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    std::puts("[PRECHECK] waiting for CRC-valid target heartbeat; no command sent yet");
    while (takeMessage(serial, parser, message, pending, observe_deadline)) {
      if (!message.crc_checked || message.msgid != ego_mav::Heartbeat::ID ||
          message.sysid != target_system || message.compid != target_component) continue;
      ego_mav::Heartbeat heartbeat;
      if (!ego_mav::Heartbeat::decode(message.payload.data(), message.payload.size(), heartbeat) || heartbeat.type == 6)
        continue;
      std::printf("[FCU] sysid=%u compid=%u autopilot=%u armed=%d main_mode=%u sub_mode=%u\n",
                  message.sysid, message.compid, heartbeat.autopilot, heartbeat.armed(),
                  heartbeat.mainMode(), heartbeat.subMode());
      if (heartbeat.armed() && !mode_only) throw std::runtime_error("refusing ARM sequence: FCU heartbeat says already armed");
      ready = true;
      break;
    }
    if (!ready) throw std::runtime_error("no valid disarmed target heartbeat within 3 seconds");

    auto sendCommand = [&](const ego_mav::CommandLong &command, uint8_t sequence, const char *label) {
      uint8_t payload[ego_mav::CommandLong::LEN];
      command.encode(payload);
      const auto frame = ego_mav::pack(source_system, source_component, sequence, ego_mav::CommandLong::ID,
                                       payload, sizeof(payload), !mavlink_v1);
      if (frame.empty() || !serial.write(frame.data(), frame.size()))
        throw std::runtime_error(std::string("failed to transmit ") + label);
      return frame.size();
    };

    if (mode_only) {
      ego_mav::Heartbeat gcs_heartbeat;
      gcs_heartbeat.type = 6;
      gcs_heartbeat.autopilot = 8;
      gcs_heartbeat.base_mode = 192;
      gcs_heartbeat.system_status = 4;
      uint8_t heartbeat_payload[ego_mav::Heartbeat::LEN];
      gcs_heartbeat.encode(heartbeat_payload);
      const auto heartbeat_frame = ego_mav::pack(source_system, source_component, 0,
                                                   ego_mav::Heartbeat::ID,
                                                   heartbeat_payload, sizeof(heartbeat_payload), !mavlink_v1);
      if (heartbeat_frame.empty() || !serial.write(heartbeat_frame.data(), heartbeat_frame.size()))
        throw std::runtime_error("failed to transmit GCS heartbeat");
      std::printf("[TX] sent GCS HEARTBEAT sysid=%u compid=%u bytes=%zu\n",
                  source_system, source_component, heartbeat_frame.size());
    } else {
      const auto arm_command = ego_mav::CommandLong::arm(true, target_system);
      const size_t arm_bytes = sendCommand(arm_command, 0, "ARM command");
      std::printf("[TX] sent one ARM command=%u target=%u/%u bytes=%zu\n",
                  ego_mav::CMD_COMPONENT_ARM_DISARM, target_system, target_component, arm_bytes);

      bool armed_seen = false;
      const auto arm_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (takeMessage(serial, parser, message, pending, arm_deadline)) {
        if (!message.crc_checked || message.sysid != target_system || message.compid != target_component) continue;
        if (message.msgid == ego_mav::CommandAck::ID) {
          ego_mav::CommandAck ack;
          if (ego_mav::CommandAck::decode(message.payload.data(), message.payload.size(), ack) &&
              ack.command == ego_mav::CMD_COMPONENT_ARM_DISARM) {
            std::printf("[ARM ACK] result=%u\n", ack.result);
            if (ack.result != 0 && ack.result != 5)
              throw std::runtime_error("FCU rejected ARM command; OFFBOARD request not sent");
          }
        } else if (message.msgid == ego_mav::Heartbeat::ID) {
          ego_mav::Heartbeat heartbeat;
          if (ego_mav::Heartbeat::decode(message.payload.data(), message.payload.size(), heartbeat) && heartbeat.type != 6) {
            std::printf("[FCU] armed=%d main_mode=%u sub_mode=%u\n",
                        heartbeat.armed(), heartbeat.mainMode(), heartbeat.subMode());
            if (heartbeat.armed()) {
              armed_seen = true;
              break;
            }
          }
        }
      }
      if (!armed_seen) throw std::runtime_error("ARM not confirmed in heartbeat; OFFBOARD request not sent");
    }

    if (mode_only) {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    const auto mode_command = ego_mav::CommandLong::setMode(ego_mav::MODE_OFFBOARD, 0, target_system);
    const size_t mode_bytes = sendCommand(mode_command, 1, "OFFBOARD mode request");
    std::printf("[TX] sent one COMMAND_LONG command=%u main_mode=%u sub_mode=0 target=%u/%u bytes=%zu arm=%d\n",
                ego_mav::CMD_DO_SET_MODE, ego_mav::MODE_OFFBOARD, target_system, target_component, mode_bytes,
                mode_only ? 0 : 1);

    {
      uint8_t mode_payload[ego_mav::CommandLong::LEN];
      mode_command.encode(mode_payload);
      const auto frame_command = ego_mav::pack(source_system, source_component, 1, ego_mav::CommandLong::ID,
                                                mode_payload, ego_mav::CommandLong::LEN, !mavlink_v1);
      std::printf("[TX HEX]");
      for (uint8_t byte : frame_command) std::printf(" %02X", byte);
      std::putchar('\n');
    }

    bool accepted_ack = false;
    bool offboard_seen = false;
    const auto confirm_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (takeMessage(serial, parser, message, pending, confirm_deadline)) {
      if (!message.crc_checked) continue;
      if (message.msgid == ego_mav::CommandAck::ID) {
        ego_mav::CommandAck ack;
        if (ego_mav::CommandAck::decode(message.payload.data(), message.payload.size(), ack) &&
            ack.command == ego_mav::CMD_DO_SET_MODE) {
          std::printf("[ACK] sysid=%u compid=%u command=%u result=%u\n",
                      message.sysid, message.compid, ack.command, ack.result);
          accepted_ack = ack.result == 0;
        }
      } else if (message.msgid == 253) {
        std::string text;
        if (!message.payload.empty()) {
          for (size_t i = 1; i < message.payload.size() && i < 51; ++i) {
            if (message.payload[i] == 0) break;
            text.push_back(static_cast<char>(message.payload[i]));
          }
        }
        std::printf("[STATUSTEXT] sysid=%u compid=%u severity=%u text=%s\n",
                    message.sysid, message.compid,
                    message.payload.empty() ? 0 : message.payload[0], text.c_str());
      } else if (message.msgid == ego_mav::Heartbeat::ID) {
        ego_mav::Heartbeat heartbeat;
        if (ego_mav::Heartbeat::decode(message.payload.data(), message.payload.size(), heartbeat) && heartbeat.type != 6) {
          std::printf("[FCU] armed=%d main_mode=%u sub_mode=%u\n",
                      heartbeat.armed(), heartbeat.mainMode(), heartbeat.subMode());
          if (heartbeat.mainMode() == ego_mav::MODE_OFFBOARD) {
            offboard_seen = true;
            break;
          }
        }
      }
    }
    if (offboard_seen) {
      std::puts("[RESULT] OFFBOARD observed in FCU heartbeat");
      if (mode_only) {
        ego_mav::SetPositionTargetLocalNed setpoint;
        setpoint.time_boot_ms = 0;
        setpoint.x = 0.0f;
        setpoint.y = 0.0f;
        setpoint.z = 0.0f;
        setpoint.vx = setpoint.vy = setpoint.vz = 0.0f;
        setpoint.afx = setpoint.afy = setpoint.afz = 0.0f;
        setpoint.yaw = 0.0f;
        setpoint.yaw_rate = 0.0f;
        setpoint.type_mask = static_cast<uint16_t>((1 << 3) | (1 << 4) | (1 << 5) |
                                                   (1 << 6) | (1 << 7) | (1 << 8) | (1 << 11));
        setpoint.target_system = target_system;
        setpoint.target_component = target_component;
        setpoint.coordinate_frame = 1;

        uint8_t setpoint_payload[ego_mav::SetPositionTargetLocalNed::LEN];
        setpoint.encode(setpoint_payload);
        std::puts("[SETPOINT] sending 5 seconds of static x=0 y=0 z=0 local-NED v1 frames; no arm/takeoff");
        const auto setpoint_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        unsigned setpoint_count = 0;
        while (std::chrono::steady_clock::now() < setpoint_deadline) {
          uint32_t boot_ms = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
          setpoint.time_boot_ms = boot_ms;
          setpoint.encode(setpoint_payload);
          const auto setpoint_frame = ego_mav::pack(source_system, source_component, 2,
                                                     ego_mav::SetPositionTargetLocalNed::ID,
                                                     setpoint_payload, sizeof(setpoint_payload));
          if (!setpoint_frame.empty() && serial.write(setpoint_frame.data(), setpoint_frame.size()))
            ++setpoint_count;

          MavMessage rx;
          while (takeMessage(serial, parser, rx, pending,
                             std::chrono::steady_clock::now() + std::chrono::milliseconds(25))) {
            if (!rx.crc_checked) continue;
            if (rx.msgid == ego_mav::Heartbeat::ID) {
              ego_mav::Heartbeat heartbeat;
              if (ego_mav::Heartbeat::decode(rx.payload.data(), rx.payload.size(), heartbeat) && heartbeat.type != 6)
                std::printf("[SETPOINT FCU] armed=%d main_mode=%u sub_mode=%u\n",
                            heartbeat.armed(), heartbeat.mainMode(), heartbeat.subMode());
            } else if (rx.msgid == ego_mav::CommandAck::ID) {
              ego_mav::CommandAck ack;
              if (ego_mav::CommandAck::decode(rx.payload.data(), rx.payload.size(), ack))
                std::printf("[SETPOINT ACK] command=%u result=%u\n", ack.command, ack.result);
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::printf("[SETPOINT RESULT] sent_v1_frames=%u; setpoints have no ACK protocol by design\n", setpoint_count);
      }
      serial.close();
      return 0;
    }
    std::printf("[RESULT] OFFBOARD not observed; accepted_ack=%d; no further commands sent\n", accepted_ack);
    return 1;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "[ERROR] %s\n", error.what());
    usage();
    return 2;
  }
}