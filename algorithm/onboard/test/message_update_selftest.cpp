// Verify continuous messages, independent snapshots and timestamp update rules.
// Only in-memory frames are used; this test never opens a device.
#include <onboard/offboard_pilot.h>
#include <cstdio>

using namespace ego_mav;
using namespace ego_onboard;
namespace {
int failures = 0;
void check(bool ok, const char *label) {
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", label);
  if (!ok) ++failures;
}
struct History : PlannerIO {
  std::vector<ego_sensor::TimedPose> poses;
  void onPose(const ego_sensor::TimedPose &p) override { poses.push_back(p); }
  void onScanFrame(const ego_sensor::LidarFrame &) override {}
  bool step(double, const Eigen::Vector3d &, ego_planner::QuadrotorCommand &) override { return false; }
};
std::vector<uint8_t> position(uint32_t boot, int32_t height, uint8_t source=1) {
  uint8_t p[28]{};
  wrU32(p, 0, boot); wrI32(p, 4, 210000000); wrI32(p, 8, 1130000000);
  wrI32(p, 16, height);
  return pack(source, 1, uint8_t(boot), 33, p, sizeof(p));
}
std::vector<uint8_t> attitude(uint32_t boot) {
  uint8_t p[28]{}; wrU32(p, 0, boot);
  return pack(1, 1, uint8_t(boot), 30, p, sizeof(p));
}
void append(std::vector<uint8_t> &stream, const std::vector<uint8_t> &frame) {
  stream.insert(stream.end(), frame.begin(), frame.end());
}
void feed(OffboardPilot &pilot, double now, const std::vector<uint8_t> &frame) {
  pilot.setReceiveTime(now); pilot.onMavlinkBytes(frame.data(), frame.size());
}
void replyTimesync(OffboardPilot &pilot, double sent, const MavMessage &message) {
  Timesync request;
  if (message.msgid != Timesync::ID ||
      !Timesync::decode(message.payload.data(), message.payload.size(), request) || request.tc1 != 0) return;
  constexpr double offset = 9.1;
  request.tc1 = int64_t((sent + 0.002 - offset) * 1e9);
  uint8_t payload[Timesync::LEN]{};
  request.encode(payload);
  feed(pilot, sent + 0.004, pack(1, 1, 0, Timesync::ID, payload, sizeof(payload)));
}
void synchronize(OffboardPilot &pilot) {
  for (int i=0; i<10; ++i) {
    const double sent = 10.0 + i * 0.21;
    std::vector<std::vector<uint8_t>> requests;
    pilot.tick(sent, requests);
    Parser parser;
    for (const auto &wire : requests) for (auto byte : wire) {
      MavMessage message;
      if (parser.push(byte, message)) replyTimesync(pilot, sent, message);
    }
  }
}
void checkHandover(const std::vector<uint8_t> &home) {
  OffboardPilot pilot;
  OffboardPilot::Config config;
  config.cruise_alt_m = 5.0;
  config.gps_min_fix = 6;
  pilot.setConfig(config);
  pilot.setInputReady(true);
  pilot.start({{21, 113, 5}, {21, 113.0001, 5}});
  synchronize(pilot);
  double now = 12.0;
  feed(pilot, now, home);
  struct Output { bool setpoint = false; bool offboard_request = false; };
  auto advance = [&](int32_t height_mm, int ticks, uint8_t mode = MODE_POS_HOLD) {
    Output output;
    Heartbeat heartbeat; heartbeat.type = 2; heartbeat.autopilot = 12;
    heartbeat.custom_mode = uint32_t(mode) << 16;
    uint8_t hb[Heartbeat::LEN]{}; heartbeat.encode(hb);
    uint8_t gps[30]{}; gps[28] = 6;
    for (int i = 0; i < ticks; ++i, now += 0.125) {
      const auto boot = pilot.timeAlignment().bootMs(now - 0.010);
      feed(pilot, now, pack(1, 1, 0, Heartbeat::ID, hb, sizeof(hb)));
      feed(pilot, now, pack(1, 1, 0, GpsRawInt::ID, gps, sizeof(gps)));
      feed(pilot, now, position(boot, height_mm));
      feed(pilot, now, attitude(boot));
      std::vector<std::vector<uint8_t>> tx;
      pilot.tick(now, tx);
      Parser parser;
      for (const auto &frame : tx) for (auto byte : frame) {
        MavMessage message;
        if (!parser.push(byte, message)) continue;
        replyTimesync(pilot, now, message);
        if (message.msgid == SetPositionTargetLocalNed::ID) output.setpoint = true;
        if (message.msgid == CommandLong::ID && message.payload.size() >= 30 &&
            rdU16(message.payload.data(), 28) == CMD_DO_SET_MODE &&
            rdF32(message.payload.data(), 4) == MODE_OFFBOARD) output.offboard_request = true;
      }
    }
    return output;
  };
  const auto takeoff = advance(5000, 14, MODE_AUTO);
  const auto below = advance(2999, 14);
  const auto above = advance(7001, 14);
  check(!takeoff.setpoint && !takeoff.offboard_request && !below.setpoint && !below.offboard_request &&
        !above.setpoint && !above.offboard_request && pilot.phase() == Phase::WAIT_GROUND_START,
        "takeoff mode and heights outside the 3–7 m window block handover");
  const auto early = advance(3000, 12);
  check(!early.setpoint && !early.offboard_request && pilot.phase() == Phase::WAIT_GROUND_START,
        "no setpoint is sent before 1.5 seconds of qualifying hover");
  const auto lower = advance(3000, 2);
  check(lower.setpoint && !lower.offboard_request && pilot.phase() == Phase::PRE_OFFBOARD,
        "the inclusive lower boundary can start the pre-Offboard stream");
  const auto drift = advance(2999, 16);
  check(!drift.setpoint && !drift.offboard_request && pilot.phase() == Phase::WAIT_GROUND_START,
        "leaving the window during pre-stream cancels it before the mode request");
  const auto upper_hover = advance(7000, 13);
  check(!upper_hover.setpoint && !upper_hover.offboard_request && pilot.phase() == Phase::PRE_OFFBOARD,
        "a fresh 1.5 second hover at the inclusive upper boundary qualifies for pre-stream");
  const auto upper_stream = advance(7000, 11);
  check(upper_stream.setpoint && !upper_stream.offboard_request,
        "the renewed pre-stream sends setpoints before requesting Offboard");
  check(advance(7000, 1).offboard_request, "Offboard is requested after the full pre-stream at the upper boundary");
  advance(7000, 1, MODE_OFFBOARD);
  check(pilot.phase() == Phase::OFFBOARD_CRUISE, "the FCU Offboard heartbeat confirms takeover");
}
}
int main() {
  OffboardPilot pilot;
  History history;
  pilot.setPlanner(&history);
  OffboardPilot::Config config;
  config.pose_timeout_s = 1.1; // Allow this batched fixture to span one second.
  pilot.setConfig(config); pilot.startObserving();
  synchronize(pilot);
  check(pilot.clockReady(13.0), "FCU TIMESYNC is established before consuming telemetry");
  uint8_t home[60]{}; wrI32(home, 0, 210000000); wrI32(home, 4, 1130000000);
  const auto home_frame = pack(1, 1, 0, HomePosition::ID, home, sizeof(home));
  feed(pilot, 13.0, home_frame);
  // CRC-valid v2 OFFBOARD heartbeats, with and without a signature trailer.
  const std::vector<uint8_t> v2_heartbeats{
      0xfd, 9, 0, 0, 0xfe, 1, 1, 0, 0, 0, 0, 0, 6, 0, 2, 12, 192, 4, 3, 0xd7, 0xaf,
      0xfd, 9, 1, 0, 0xfe, 1, 1, 0, 0, 0, 0, 0, 6, 0, 2, 12, 192, 4, 3, 0x30, 0x57,
      0xfe, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  feed(pilot, 13.0, v2_heartbeats);
  check(!pilot.armed() && pilot.mainMode() == 0, "v2 frames do not update the FCU state on the v1 link");
  std::vector<uint8_t> stream;
  for (uint32_t i=0; i<100; ++i) {
    append(stream, position(3000+i*10, 50000+int32_t(i)*10));
    append(stream, attitude(3000+i*10));
  }
  // One receive chunk deliberately contains 200 adjacent complete messages.
  feed(pilot, 13.1, stream);
  check(history.poses.size() == 100, "200 adjacent messages are all parsed, without replacement or concatenation damage");
  bool ordered = history.poses.size() == 100;
  for (size_t i=0; i<history.poses.size(); ++i)
    ordered = ordered && std::abs(history.poses[i].p.z()-(50+i*.01)) < 1e-9;
  check(ordered && std::abs(pilot.latestPose().p.z()-50.99) < 1e-9,
        "current pose is the newest sample while all earlier pose copies retain their values");
  const auto saved = pilot.latestPose();
  feed(pilot, 13.11, position(3990, 90000));
  feed(pilot, 13.11, attitude(3990));
  feed(pilot, 13.12, position(3980, 80000));
  feed(pilot, 13.12, attitude(3980));
  check(history.poses.size() == 100 && (pilot.latestPose().p-saved.p).norm() == 0,
        "duplicate and slightly out-of-order position/attitude cannot overwrite newer state");
  feed(pilot, 13.13, position(4000, 70000, 2));
  check((pilot.latestPose().p-saved.p).norm() == 0,
        "another aircraft's message cannot replace this aircraft's position");
  auto damaged = position(2000, 70000); damaged[14] ^= 1;
  feed(pilot, 13.14, damaged);
  check(pilot.crcErrors() > 0 && (pilot.latestPose().p-saved.p).norm() == 0,
        "CRC-damaged message cannot replace the valid state");
  feed(pilot, 13.15, position(4000, 51000));
  feed(pilot, 13.15, attitude(4000));
  check(history.poses.size() == 101 && std::abs(pilot.latestPose().p.z()-51) < 1e-9 &&
        std::abs(history.poses.front().p.z()-50) < 1e-9,
        "next valid update recovers and leaves previous samples intact");
  feed(pilot, 13.16, position(3000, 10000));
  check(pilot.phase() == Phase::FAILED && std::abs(pilot.latestPose().p.z()-51) < 1e-9,
        "large boot-time rollback fails explicitly instead of overwriting with an old position");

  ego_sensor::PoseBuffer poses;
  poses.setMaxSize(3);
  ego_sensor::TimedPose pose;
  for (int i=0; i<4; ++i) { pose.t=i; pose.p.x()=i; poses.add(pose); }
  ego_sensor::TimedPose out;
  check(!poses.poseAt(0, out) && poses.poseAt(3, out) && out.p.x()==3,
        "bounded pose history evicts oldest entries and retains newest entries");
  pose.t=2; pose.p.x()=99; poses.add(pose);
  check(poses.poseAt(3, out) && out.p.x()==3, "older pose insertion does not replace the newest history entry");
  pose.t=3; pose.p.x()=4; poses.add(pose);
  check(poses.poseAt(3, out) && out.p.x()==4, "same-time pose correction deliberately replaces its matching history entry");
  checkHandover(home_frame);
  std::printf("message_update_selftest: %d failures\n", failures);
  return failures ? 1 : 0;
}
