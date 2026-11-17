#ifndef ONBOARD_KML_PLANNER_START_H
#define ONBOARD_KML_PLANNER_START_H
#include <onboard/mission_input.h>
namespace ego_onboard {
// 作用：把已校验的 KML 航线接到规划器上，并在允许的阶段提前预热轨迹；
// 这里只调用 PlannerIO，不下发任何飞控报文。

// 把 KML 航线（经纬高，高度相对起飞点）转成局部 ENU：
inline bool bindKmlPlanner(const std::vector<GeoWaypoint> &route,const ego_mav::GeoOrigin &home,PlannerIO &planner) {
  if(route.size()<2||!home.valid)return false;
  std::vector<Eigen::Vector3d> enu;enu.reserve(route.size());
  for(const auto &w:route){auto xy=ego_mav::geoToEnuXY(w.lat_deg,w.lon_deg,home);enu.emplace_back(xy.x(),xy.y(),w.alt_rel_m);}
  planner.setReferenceRoute(enu);planner.setLeg(enu[0],enu[1],enu.size()==2);return true;
}
// 预规划：只有在"链路建立"或"等地面站起飞"且 KML/航线/输入都就绪时才跑一拍，
// 目的是让规划器提前产出轨迹，接管时不必再等首次规划。返回值只表示是否执行了预热。
inline bool preplanKml(bool kml_ready,bool route_bound,bool inputs_ready,Phase phase,double now,
                      const Eigen::Vector3d &position,PlannerIO &planner) {
  if(!kml_ready||!route_bound||!inputs_ready)return false;
  if(phase!=Phase::LINK&&phase!=Phase::WAIT_GROUND_START)return false;
  ego_planner::QuadrotorCommand preview;planner.step(now,position,preview);return true;
}
}
#endif
