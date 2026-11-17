#ifndef _SENSOR_LIDAR_TYPES_H_
#define _SENSOR_LIDAR_TYPES_H_

#include <Eigen/Eigen>

#include <cstddef>
#include <cmath>
#include <deque>
#include <vector>

namespace ego_sensor
{
  // 感知输入的边界（这就是本模块存在的意义）
  // 上游只负责给出两样东西：
  //   1. 带时间戳的位姿   —— 实机上由飞控（姿态）+ GPS/RTK（位置）融合得到
  //   2. 一帧原始激光点云 —— 实机上由 Livox-SDK2 的回调给出（尚未转世界系）
  struct PositionAnchor
  {
    double t = 0.0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    double max_extrapolation_s = 0.25;
    bool valid = false;

    bool positionAt(double target, Eigen::Vector3d &out) const
    {
      const double dt = target - t;
      if (!valid || !std::isfinite(target) || !std::isfinite(t) ||
          !std::isfinite(max_extrapolation_s) || max_extrapolation_s <= 0 ||
          !p.allFinite() || !v.allFinite() ||
          dt < -1e-9 || dt > max_extrapolation_s + 1e-9) return false;
      // 仅消除浮点舍入产生的微小负值，不把过期的dt截短后伪装成目标时刻。
      out = p + v * (dt < 0 ? 0 : dt);
      return out.allFinite();
    }
  };

  // 一帧位姿采样，局部 ENU / 世界系。
  struct TimedPose
  {
    double t = 0.0;                                        // 秒，与 LidarPoint::t_rel / LidarFrame::t_base 同一时钟
    Eigen::Vector3d p = Eigen::Vector3d::Zero();            // 世界系位置
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // 机体系 -> 世界系
    PositionAnchor position_anchor; // 飞控位置+速度来源；逐点位置直接由该真实锚点外推
  };

  // 时间戳位姿缓冲 + 插值。
  class PoseBuffer
  {
  public:
    void add(const TimedPose &pose);
    bool poseAt(double t, TimedPose &out) const;
    bool empty() const { return buf_.empty(); }
    double oldestTime() const { return buf_.empty() ? 0.0 : buf_.front().t; }
    double newestTime() const { return buf_.empty() ? 0.0 : buf_.back().t; }
    size_t size() const { return buf_.size(); }
    void setMaxSize(size_t n) { max_size_ = n; }
    void setStrictCoverage(bool strict) { strict_ = strict; }
    void setMaxGap(double seconds) { max_gap_ = seconds; }

  private:
    std::deque<TimedPose> buf_;
    size_t max_size_ = 500;
    bool strict_ = false;
    double max_gap_ = 0.1;
  };

  struct LidarPoint
  {
    Eigen::Vector3d p = Eigen::Vector3d::Zero(); // 传感器系，米
    double t_rel = 0.0;                          // 相对本帧基准时刻（秒）—— 对应 CustomPoint::offset_time
    float reflectivity = 0.0f;                   // 0~255
    uint8_t tag = 0;                             // Livox tag
    uint8_t line = 0;                            // 激光线号（Mid 系列无，固定 0）
  };

  // 一帧原始点云：未去畸变、未转世界系。
  struct LidarFrame
  {
    double t_base = 0.0; // 本帧基准时刻 = 第一个包的雷达时间（位姿时钟）。
    bool payload_crc_present = true; // false when any packet omits its payload CRC
    uint8_t lidar_id = 0; // 多雷达时的设备号
    int data_type = 0;    // LivoxLidarPointDataType，仅用于排查
    uint8_t time_type = 0; // 0=unsynced, 1=PTP, 2=GPS
    std::vector<LidarPoint> points;
  };

  // 外参：传感器 -> 机体系
  struct Extrinsic
  {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
  };

} // namespace ego_sensor

#endif
