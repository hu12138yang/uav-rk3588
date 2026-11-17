// Offline integration: synthetic FCU telemetry/cloud -> real AsyncPlanner/EGO
// -> real OffboardPilot -> CRC-checked MAVLink setpoints. No device is opened.
#include <onboard/async_planner.h>
#include <onboard/config_file.h>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <malloc.h>
#include <sys/resource.h>
#include <thread>
#include <unistd.h>

using namespace ego_onboard;
using namespace ego_mav;
namespace {
int failures = 0;
void check(bool ok, const char *label) {
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", label);
  if (!ok) ++failures;
}
double rssMiB() {
  long total = 0, resident = 0;
  std::ifstream stat("/proc/self/statm"); stat >> total >> resident;
  return resident * double(sysconf(_SC_PAGESIZE)) / (1024 * 1024);
}
// This only audits calls; every pose, scan, planning result and health decision
// comes from the production worker, not a test planner implementation.
struct AuditedPlanner : PlannerIO {
  AsyncPlanner worker;
  ego_planner::QuadrotorCommand last;
  uint64_t ready = 0, misses = 0;
  bool stepped = false, usable = false;
  explicit AuditedPlanner(const EgoPlannerAdapterConfig &cfg) : worker(cfg, 50) {}
  void onPose(const ego_sensor::TimedPose &p) override { worker.onPose(p); }
  void onScanFrame(const ego_sensor::LidarFrame &f) override { worker.onScanFrame(f); }
  void setLeg(const Eigen::Vector3d &a, const Eigen::Vector3d &b, bool end) override { worker.setLeg(a,b,end); }
  void dropTrajectory() override { worker.dropTrajectory(); }
  void resetClockState() override { worker.resetClockState(); }
  bool sensingHealthy() const override { return worker.sensingHealthy(); }
  bool step(double now, const Eigen::Vector3d &p, ego_planner::QuadrotorCommand &cmd) override {
    stepped = true; usable = worker.step(now,p,cmd); last = cmd;
    if (usable && cmd.trajectory_flag == ego_planner::TRAJECTORY_STATUS_READY) ++ready;
    if (!usable) ++misses;
    return usable;
  }
};
void feed(OffboardPilot &pilot, double now, uint32_t id, const uint8_t *p, size_t n) {
  const auto bytes = pack(1,1,0,id,p,n);
  pilot.setReceiveTime(now); pilot.onMavlinkBytes(bytes.data(), bytes.size());
}
}

int main(int argc, char **argv) {
  if (argc == 3 && std::string(argv[2]) == "--astar-lifetime") {
    GridMap::Ptr map(new GridMap);
    map->initMap(.25,Eigen::Vector3d(2,2,2),Eigen::Vector3d(1,1,1),.25,2,0,2,"world");
    const auto baseline = mallinfo().uordblks;
    {
      AStar search;
      search.initGridMap(map,Eigen::Vector3i(40,40,40));
      const auto warm = mallinfo().uordblks;
      for (int i = 0; i < 24; ++i) search.initGridMap(map,Eigen::Vector3i(40,40,40));
      check(mallinfo().uordblks <= warm + 1024*1024,"reinitializing A* releases the previous node pool and pointer arrays");
    }
    check(mallinfo().uordblks <= baseline + 1024*1024,"destroying A* releases its complete pool");
    return failures ? 1 : 0;
  }
  if (argc != 2) return 2;
  AppConfig app; std::string error;
  if (!loadConfigFile(argv[1],app,error)) { std::puts(error.c_str()); return 2; }
  app.adapter.strict_pose_coverage = true;
  AuditedPlanner planner(app.adapter);
  OffboardPilot pilot; pilot.setConfig(app.pilot); pilot.setPlanner(&planner);
  constexpr double lat = 21, lon = 113, radians = kPi / 180;
  const double meters_per_degree = GeoOrigin::kR * radians;
  pilot.start({{lat,lon,50},{lat,lon + 200 / (meters_per_degree * std::cos(lat*radians)),50}});
  const double begin = monotonicSeconds(), boot_origin = begin - 10;
  Eigen::Vector3d position(0,0,50), velocity = Eigen::Vector3d::Zero();
  uint8_t mode = MODE_POS_HOLD;
  double next_scan = begin, next_report = begin, maximum_deviation = 0;
  uint64_t matched = 0, mismatched = 0, invalid_wire = 0, scans = 0;
  bool collision_free = true, entered_cruise = false, passed_wall = false;
  Parser parser;
  while (monotonicSeconds() - begin < 25) {
    const double now = monotonicSeconds();
    const auto boot = uint32_t((now - boot_origin) * 1000);
    Heartbeat hb; hb.type = 2; hb.autopilot = 12; hb.custom_mode = uint32_t(mode) << 16;
    uint8_t heartbeat[Heartbeat::LEN]{}; hb.encode(heartbeat);
    feed(pilot,now,Heartbeat::ID,heartbeat,sizeof(heartbeat));
    uint8_t home[60]{}; wrI32(home,0,int32_t(lat*1e7)); wrI32(home,4,int32_t(lon*1e7));
    feed(pilot,now,HomePosition::ID,home,sizeof(home));
    uint8_t gps[30]{}; gps[28] = 6; gps[29] = 20;
    feed(pilot,now,GpsRawInt::ID,gps,sizeof(gps));
    uint8_t global[28]{}; wrU32(global,0,boot);
    wrI32(global,4,int32_t(std::llround((lat + position.y()/meters_per_degree)*1e7)));
    wrI32(global,8,int32_t(std::llround((lon + position.x()/(meters_per_degree*std::cos(lat*radians)))*1e7)));
    wrI32(global,16,int32_t(std::llround(position.z()*1000)));
    wrU16(global,20,uint16_t(int16_t(std::llround(velocity.y()*100))));
    wrU16(global,22,uint16_t(int16_t(std::llround(velocity.x()*100))));
    wrU16(global,24,uint16_t(int16_t(std::llround(-velocity.z()*100))));
    feed(pilot,now,GlobalPositionInt::ID,global,sizeof(global));
    uint8_t attitude[28]{}; wrU32(attitude,0,boot); wrF32(attitude,12,kPi/2);
    feed(pilot,now,Attitude::ID,attitude,sizeof(attitude));
    if (pilot.poseFresh(now) && now >= next_scan) {
      next_scan = now + app.lidar.frame_duration_s;
      const auto &pose = pilot.latestPose();
      ego_sensor::LidarFrame frame; frame.t_base = pose.t;
      frame.points.reserve(20000);
      // The first local target (70,0,50) is clear. A wall at x=25 blocks
      // the middle of the straight seed, requiring EGO's actual A*/optimizer.
      for (int i = 0; i < 20000; ++i) {
        const Eigen::Vector3d world(25, -2 + (i%9)*0.5, 47 + ((i/9)%11)*0.5);
        ego_sensor::LidarPoint point;
        point.p = pose.q.conjugate() * (world - pose.p);
        frame.points.push_back(point);
      }
      planner.onScanFrame(frame); ++scans;
    }
    pilot.setInputReady(planner.sensingHealthy());
    planner.stepped = false;
    std::vector<std::vector<uint8_t>> tx; pilot.tick(now,tx);
    for (const auto &wire : tx) for (const auto byte : wire) {
      MavMessage message;
      if (!parser.push(byte,message)) continue;
      if (!message.crc_checked) { ++invalid_wire; continue; }
      if (message.msgid == Timesync::ID) {
        Timesync sync;
        if (Timesync::decode(message.payload.data(),message.payload.size(),sync) && sync.tc1 == 0) {
          sync.tc1 = int64_t((double(sync.ts1)*1e-9 - boot_origin)*1e9);
          sync.target_system = 245; sync.target_component = 1;
          uint8_t p[Timesync::LEN]{}; sync.encode(p);
          feed(pilot,now,Timesync::ID,p,sizeof(p));
        }
      } else if (message.msgid == CommandLong::ID) {
        uint8_t p[CommandLong::LEN]{};
        padPayload(message.payload.data(),message.payload.size(),p,sizeof(p));
        if (rdU16(p,28) == CMD_DO_SET_MODE && rdF32(p,4) == MODE_OFFBOARD) mode = MODE_OFFBOARD;
      } else if (message.msgid == SetPositionTargetLocalNed::ID) {
        uint8_t p[SetPositionTargetLocalNed::LEN]{};
        padPayload(message.payload.data(),message.payload.size(),p,sizeof(p));
        const Eigen::Vector3d enu(rdF32(p,8),rdF32(p,4),-rdF32(p,12));
        if (planner.stepped && planner.usable && planner.last.trajectory_flag == ego_planner::TRAJECTORY_STATUS_READY) {
          const bool same = (enu-planner.last.position).norm() < 1e-4 &&
              std::abs(rdF32(p,40)-yawEnuToNed(planner.last.yaw)) < 1e-5 && rdU16(p,48) == 0x09F8;
          if (same) ++matched; else ++mismatched;
          position = enu; velocity = planner.last.velocity;
          maximum_deviation = std::max(maximum_deviation,std::hypot(position.y(),position.z()-50));
          // Independent physical-box check, including a conservative voxel pad.
          const double padding = app.adapter.obstacles_inflation + app.adapter.resolution;
          if (std::abs(position.x()-25) <= padding && std::abs(position.y()) <= 2+padding &&
              position.z() >= 47-padding && position.z() <= 52+padding) collision_free = false;
        }
      }
    }
    entered_cruise = entered_cruise || pilot.phase() == Phase::OFFBOARD_CRUISE;
    if (now >= next_report) {
      next_report = now + 1;
      const auto st = planner.worker.stats();
      std::printf("[LOAD] t=%.1f phase=%s pos=(%.2f,%.2f,%.2f) rss=%.1fMiB plans=%llu opt=%llu astar=%llu/%llu matched=%llu misses=%llu\n",
        now-begin,phaseName(pilot.phase()),position.x(),position.y(),position.z(),rssMiB(),
        (unsigned long long)st.plan_success,(unsigned long long)st.ego_optimizations,
        (unsigned long long)st.astar_successes,(unsigned long long)st.astar_searches,
        (unsigned long long)matched,(unsigned long long)planner.misses);
    }
    if (position.x() > 45 && maximum_deviation > 3) { passed_wall = true; break; }
    if (pilot.phase() == Phase::FAILED) { std::printf("[ERROR] %s\n",pilot.lastError()); break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto st = planner.worker.stats();
  rusage memory{}; getrusage(RUSAGE_SELF,&memory);
  std::printf("[EGO OUTPUT] scans=%llu points=%ld success=%llu fail=%d optimizations=%llu astar=%llu/%llu matched=%llu mismatched=%llu deviation=%.2fm rss_peak=%.1fMiB node_bytes=%zu point_bytes=%zu\n",
    (unsigned long long)scans,st.cloud_points,(unsigned long long)st.plan_success,st.plan_fail,
    (unsigned long long)st.ego_optimizations,(unsigned long long)st.astar_successes,(unsigned long long)st.astar_searches,
    (unsigned long long)matched,(unsigned long long)mismatched,maximum_deviation,memory.ru_maxrss/1024.0,
    sizeof(GridNode),sizeof(ego_sensor::LidarPoint));
  check(entered_cruise,"real pilot completes clock/GPS/perception gates and enters Offboard");
  check(st.ego_optimizations > 0 && st.astar_successes > 0,"EGO optimizer and obstacle A* searches actually execute");
  check(matched > 50 && mismatched == 0 && invalid_wire == 0,"CRC-valid outgoing setpoints match EGO positions and yaw after ENU/NED conversion");
  check(pilot.plannerSetpoints() == matched,"production EGO setpoint counter agrees with independently decoded outgoing frames");
  check(collision_free && passed_wall,"emitted setpoints deviate from the blocked straight route and pass the inflated wall");
  check(st.cloud_points > 100000 && st.inflate_samples > 10,"continuous 20k-point frames enter the real async map worker");
  return failures ? 1 : 0;
}
