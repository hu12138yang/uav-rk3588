#ifndef _ONBOARD_CONFIG_FILE_H_
#define _ONBOARD_CONFIG_FILE_H_

// 代码里的结构体默认值只是兜底，实机一律以 YAML 为准
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "onboard/ego_planner_adapter.h"
#include "onboard/offboard_pilot.h"
#include "onboard/mission_input.h"
#include "onboard/livox_source.h"

namespace ego_onboard
{
  struct AppConfig
  {
    OffboardPilot::Config pilot;          // 飞控时序/安全阈值
    MissionInputConfig mission_input;     // KML 等待与解析
    LivoxSource::Config lidar;            // 雷达帧组装与过滤
    EgoPlannerAdapterConfig adapter;      // 建图与局部规划
    std::string uplink;                   // 状态上报地址 "ip:port"，空串 = 不上报
    // 机体包络半径
    double vehicle_radius = 0.85;
    // 规划器要求的障碍物最小间隙（膨胀后的安全距离，须 >= vehicle_radius）
    double min_obstacle_gap = 1.5; 
  };

  // 读 YAML 并填充 out；缺省的键保持结构体默认值。
  bool loadConfigFile(const std::string &path, AppConfig &out, std::string &err);

} // namespace ego_onboard

#endif
