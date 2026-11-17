#ifndef ONBOARD_MISSION_INPUT_H
#define ONBOARD_MISSION_INPUT_H

#include <cstddef>
#include <string>
#include <vector>

#include "onboard/offboard_pilot.h"

namespace ego_onboard
{
  // One local KML task per run. Optional inbox waiting precedes device opening.
  // The uploader overwrites the configured path directly; there is no settle
  // window, so the file is read as soon as it exists. READY latches the task.
  //
  // 一次开机只接受一个本地 KML 任务：wait_for_file=true 时先等文件出现，
  // 出现后立即读取、解析、校验，通过即进入 READY（任务锁定，不再换文件）。
  // 上传方直接用新文件覆盖同一路径，不要求原子重命名。
  struct MissionInputConfig
  {
    std::string file;                   // KML 路径（相对启动工作目录）
    bool wait_for_file = false;         // 文件不存在时是否一直等待（开机接任务模式）
    std::size_t max_bytes = 5 * 1024 * 1024; // 文件大小上限，超限直接拒绝
    std::size_t max_waypoints = 2000;    // includes the first takeoff point
    std::string path_name;              // select a LineString Placemark by name
    std::string height_mode = "constant"; // project default: fixed 50 m above HOME
    double home_amsl_m = 0.0;           // HOME 海拔（相对高度换算成绝对高度时用）
    bool have_home_amsl = false;        // 是否已知 HOME 海拔
    double constant_alt_m = 50.0;       // height_mode=constant 时的相对高度
    double min_alt_m = 0.5;             // all relative heights must exceed this
    double max_alt_m = 1000.0;          // 相对高度上限
  };

  // 校验通过后的任务：route[0] 是起飞点，之后是导航点
  struct PreparedMission
  {
    std::vector<GeoWaypoint> route;     // route[0] = takeoff, route[1..] = NAV
    std::vector<std::string> warnings;  // 非致命问题（如高度来源推断）
    std::string height_source;          // 高度取自哪里（KML 绝对高度/配置常量等）
    bool return_leg_added = false;      // 是否自动追加了返航段
  };

  // 任务输入状态机：IDLE -> WAIT_KML -> LOADING_KML -> PARSING_KML
  //               -> VALIDATING_ROUTE -> READY / FAILED
  enum class MissionInputState
  {
    IDLE, WAIT_KML, LOADING_KML, PARSING_KML, VALIDATING_ROUTE, READY, FAILED
  };
  const char *missionInputStateName(MissionInputState state);

  // Strict XML parser for local KML input. It does not fetch
  // URLs, load external entities, or guess a terrain-to-HOME height offset.
  // 解析成功后只填 mission，不做任何设备操作。
  bool parseMissionKml(const std::string &xml, const MissionInputConfig &config,
                       PreparedMission &mission, std::string &error);
  // 航点数量、相对高度范围、坐标合法性等业务校验
  bool validateMissionRoute(const PreparedMission &mission,
                            const MissionInputConfig &config, std::string &error);

  class MissionInput
  {
  public:
    void start(const MissionInputConfig &config);
    void step();                      // 用内部单调时钟推进一步
    void step(double monotonic_now);  // Explicit clock for deterministic inbox tests.
    MissionInputState state() const { return state_; }
    const PreparedMission &mission() const { return mission_; }
    const std::string &error() const { return error_; }
  private:
    MissionInputConfig config_;
    MissionInputState state_ = MissionInputState::IDLE;
    PreparedMission mission_;
    std::string body_, error_;              // 文件内容 / 最近一次错误
    std::string observed_version_, rejected_version_; // 文件版本（用于识别覆盖/避免重试同一份坏文件）
    void rejectOrFail();                    // 解析/校验失败：回到等待（wait_for_file）或失败
  };
}
#endif
