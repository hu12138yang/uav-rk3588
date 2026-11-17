#ifndef ONBOARD_ROUTE_UTILS_H
#define ONBOARD_ROUTE_UTILS_H

#include <cmath>
#include <string>
#include "onboard/offboard_pilot.h"

namespace ego_onboard
{
  // First KML coordinate is the agreed takeoff origin. Return is an ordinary
  // NAV leg at cruise height, never a firmware RTL / land command.
  //
  // 作用：保证航线"回到起点"。若首尾已是同一点，只把末点对齐到首点；
  // 否则在末尾追加一个等于首点的航点（added=true）。容量不足时返回错误。
  inline bool ensureReturnToStart(std::vector<GeoWaypoint> &route, std::size_t max_points,
                                 bool &added, std::string &error)
  {
    added = false;
    // 至少要有"起点 + 一个任务点"，且不能超过航点上限
    if (route.size() < 2 || route.size() > max_points)
    { error = "Route must contain an origin and at least one task waypoint"; return false; }
    const auto &a = route.front();
    const auto &b = route.back();
    // 端点必须是有限值且在合法经纬范围内，避免把 NaN/越界值送进规划
    if (!std::isfinite(a.lat_deg) || !std::isfinite(a.lon_deg) || !std::isfinite(a.alt_rel_m) ||
        !std::isfinite(b.lat_deg) || !std::isfinite(b.lon_deg) || !std::isfinite(b.alt_rel_m) ||
        std::abs(a.lat_deg) > 90 || std::abs(b.lat_deg) > 90 ||
        std::abs(a.lon_deg) > 180 || std::abs(b.lon_deg) > 180)
    { error = "Invalid route endpoints"; return false; }
    // Compare at the degE7 precision used by the FCU position telemetry.
    // 用飞控遥测的 degE7 精度比较，避免浮点尾差把"已闭环"误判成"未闭环"。
    const bool same = std::llround(a.lat_deg * 1e7) == std::llround(b.lat_deg * 1e7) &&
                      std::llround(a.lon_deg * 1e7) == std::llround(b.lon_deg * 1e7) &&
                      std::abs(a.alt_rel_m - b.alt_rel_m) < 0.01;
    if (same) route.back() = route.front();
    else
    {
      // 追加返航点前再确认还有容量（max_points 含首点）
      if (route.size() >= max_points)
      { error = "No waypoint capacity left for return to origin"; return false; }
      const GeoWaypoint origin = route.front();
      route.push_back(origin);
      added = true;
    }
    return true;
  }
}
#endif
