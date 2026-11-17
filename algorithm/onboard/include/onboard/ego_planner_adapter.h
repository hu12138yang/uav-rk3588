#ifndef _ONBOARD_EGO_PLANNER_ADAPTER_H_
#define _ONBOARD_EGO_PLANNER_ADAPTER_H_

// 把 ego_planner_core（EGO 局部规划 + 滚动占据栅格）接到 OffboardPilot 的 PlannerIO 上。
// 这是"我们的系统"与"飞控协议层"之间**唯一**的接线点：
//   飞控位姿     ──onPose──────► 位姿缓冲（供逐点去畸变）
//   雷达原始帧   ──onScanFrame─► 逐点去畸变 ─► 世界系点云 ─► GridMap（insert-only 滚动窗口）
//   当前航段 A/B ──setLeg──────► A→B 直线参考，内部切局部目标
//   100 Hz       ──step───────► 需要时重规划 ─► TrajectoryTracker ─► QuadrotorCommand

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>
#include <ego_core/ego_local_planner.h>
#include <ego_core/trajectory_tracker.h>
#include <plan_env/grid_map.h>
#include <sensor/lidar_types.h>
#include <sensor/scan_assembler.h>
#include "onboard/offboard_pilot.h" // PlannerIO

namespace ego_onboard
{

  struct EgoPlannerAdapterConfig
  {
    /* ---- 滚动占据栅格 ---- */
    double resolution = 0.2;              // 体素边长
    double altitude_band_half = 10.0;     // 高度带向上半宽
    double max_descent_below_ref = 2.0;   // 允许低于高度基准的最大下探
    double map_margin_z = 2.0;            // 高度带外再留的余量
    double min_flight_height = 5.0;       // 最低飞行高度（相对起飞点）
    Eigen::Vector2d map_size_xy = Eigen::Vector2d(60.0, 60.0); // 滚动窗口 x/y 尺寸
    double occupancy_update_range_xy = 9.0; // 只有机体附近的体素会被刷新
    double occupancy_update_range_z = 6.0;  // 同上，z 方向的刷新范围
    double obstacles_inflation = 1.0;     // 必须 >= 机体包络半径
    int local_map_margin = 10;            // 规划器内部局部地图边缘格数
    double recenter_margin = 8.0;         // 机体离窗口中心超过此值才滑动窗口
    int sensing_drop_frames = 10;         // 连续多少帧没进图判为感知失效
    double sensing_timeout_s = 0.5;       // 多久没有新点云进图判为感知过期
    bool strict_pose_coverage = false;    // true=每帧都必须有完整位姿覆盖，否则允许部分覆盖

    /* ---- 规划器 ---- */
    double max_vel = 5.0;                 // 巡航速度，EGO 一直以它跑
    double max_acc = 6.0;
    double planning_horizon = 6.0;        // 局部目标前视距离
    double local_target_step = 1.0;       // 落点被占时沿航线前走的步长
    double local_target_max_ratio = 1.5;  // 前视上限 = ratio * planning_horizon
    double replan_period = 0.5;           // 周期性重规划
    double goal_tolerance = 0.5;          // 到达局部目标的判定
    double obstacle_clearance = 0.5;      // 优化器里的 dist0 / swarm_clearance
    double ctrl_pt_dist = 0.4;            // B 样条控制点间距（米）
    Eigen::Vector3i astar_pool_size = Eigen::Vector3i(100, 100, 100); // A* 搜索池尺寸

    /* ---- 输出跟踪 ---- */
    double time_forward = 1.0;            // TrajectoryTracker 前视（yaw 平滑）

    /* ---- 外参：雷达 -> 机体 ---- */
    ego_sensor::Extrinsic extrinsic;
  };

  class EgoPlannerAdapter : public PlannerIO
  {
  public:
    EgoPlannerAdapter();
    ~EgoPlannerAdapter() override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 只保存配置；真正建图/建规划器在 prepareMap 时完成
    void init(const EgoPlannerAdapterConfig &cfg);

    // 新任务前清空地图与轨迹
    void reset();
    // 时钟不连续：只清位姿历史与轨迹，保留栅格地图（地图像素是世界系坐标）
    void clearPoseHistory();
    // 以给定高度基准创建滚动栅格地图和规划器；基准变化时可重复调用（重建）
    void prepareMap(double reference_height);
    // 该帧扫描结束时是否已有前后两帧位姿覆盖（逐点去畸变的前提）
    bool frameCovered(const ego_sensor::LidarFrame &frame) const;

    /* ---------------- PlannerIO ---------------- */

    // 绑定当前航段 A->B（局部 ENU）；is_last 表示到达 B 后任务结束
    void setLeg(const Eigen::Vector3d &start, const Eigen::Vector3d &end, bool is_last) override;
    // 飞控位姿入位姿缓冲，供点云逐点去畸变按时间查找
    void onPose(const ego_sensor::TimedPose &pose) override;
    // 原始雷达帧：逐点去畸变 -> 世界系点云 -> 插入滚动栅格
    void onScanFrame(const ego_sensor::LidarFrame &frame) override;
    // 推进一拍：必要时重规划并输出跟踪指令；无可用轨迹时返回 false
    bool step(double now, const Eigen::Vector3d &cur_pos,
              ego_planner::QuadrotorCommand &cmd) override;

    // 丢弃当前轨迹：丢星悬停后恢复时用
    void dropTrajectory() override;

    // 感知是否健康（连续丢帧 → 失效）。供状态机做安全悬停判断。
    bool sensingHealthy() const override;

    /* ---------------- 诊断 ---------------- */

    struct Stats
    {
      long frames_total = 0;   // 收到的原始帧
      long frames_dropped = 0; // 位姿覆盖不足而丢掉的帧
      long cloud_points = 0;   // 累计进图点数
      uint64_t step_calls = 0;          // step() 被调用的次数
      uint64_t blocked_start_holds = 0; // 起点被障碍物占用而保持的次数
      bool start_blocked = false;       // 当前起点是否被占用
      int replan = 0;          // 重规划次数
      int plan_fail = 0;       // 规划失败次数
      uint64_t plan_success = 0;
      uint64_t ego_optimizations = 0;
      uint64_t astar_searches = 0;
      uint64_t astar_successes = 0;
      uint64_t tracking_commands = 0; // 真正从 EGO 样条跟踪器取出的指令
      uint64_t hold_commands = 0;
      bool trajectory_active = false;
      int recenter = 0;        // 滑动窗口次数
      int consecutive_dropped = 0; // 当前连续丢帧数
      bool sensing_dead = false;   // 连续丢帧超阈值 → 感知失效
      double last_plan_ms = 0.0; // 最近一次规划耗时
      double plan_ms_max = 0.0;  // 规划耗时峰值
      // 地图插入耗时
      double inflate_ms_sum = 0.0;
      double inflate_ms_max = 0.0;
      int inflate_samples = 0;
    };
    const Stats &stats() const { return stats_; }

    GridMap::Ptr gridMap() const { return grid_map_; }                // 滚动栅格地图
    ego_sensor::ScanAssembler &scanAssembler() { return scan_asm_; }  // 帧组装器（诊断用）
    bool haveTrajectory() const { return have_traj_; }
    bool braking() const { return braking_; }                         // 当前是否处于刹车保持
    // 地图是否已经建好
    bool mapReady() const { return grid_map_ != nullptr; }
    // 高度基准（相对起飞点）
    double altitudeRef() const { return z_ref_; }

  private:
    // 用给定的高度基准建图 + 建规划器（调用方必须已持有 mtx_）
    void buildMap(double z_ref);
    // 清空轨迹/命令/统计等运行态，但保留地图与规划器
    void clearState();

    // 沿参考航线的弧长：投影 / 取点 / 取方向
    // 把当前位置投影到 [s_lo,s_hi] 这段航线上，返回横向距离并输出弧长 s_out
    double projectRoute(double s_lo, double s_hi, const Eigen::Vector3d &p, double &s_out) const;
    Eigen::Vector3d routeAt(double s) const;      // 弧长 s 处的航点位置
    Eigen::Vector3d routeDirAt(double s) const;   // 弧长 s 处的航向单位向量
    // 从当前弧长往前取局部目标；落点被占时逐步前移，输出实际取到的弧长
    Eigen::Vector3d localTargetAt(double s_cur, double &s_target) const;
    // 预判前方轨迹是否撞障碍（用于提前刹车）
    bool trajCollidesAhead(double now);

    EgoPlannerAdapterConfig cfg_;
    mutable std::mutex mtx_;
    bool inited_ = false;
    double z_ref_ = 0.0; // 高度基准 = 开始导航那一帧的实际高度（相对起飞点）

    GridMap::Ptr grid_map_;
    ego_planner::EgoLocalPlanner planner_;
    ego_planner::TrajectoryTracker tracker_;
    ego_sensor::ScanAssembler scan_asm_;
    ego_sensor::PoseBuffer pose_buf_;

    // 当前航段 A→B（两个点）+ 累计弧长
    std::vector<Eigen::Vector3d> route_;   // 参考航线（局部 ENU，含起飞点）
    std::vector<double> route_cum_;        // route_ 各点的累计弧长
    double route_total_ = 0.0;             // 整条航线总长

    // 当前航段在 route_ 上的范围（两段点固定为 [0, total]）
    double leg_start_s_ = 0.0;
    double leg_end_s_ = 0.0;
    bool leg_last_ = false; // 本段是最后一段（到达 B 后任务完成）

    // 跟踪状态
    bool have_traj_ = false;   // 当前是否有可跟踪的轨迹
    bool braking_ = true;      // true = 无轨迹或危险，输出刹车指令
    ego_planner::BsplineTrajectory traj_;
    double last_plan_ = -1e9;   // 上一次成功规划的时刻
    double last_safety_ = -1e9; // 上一次前向碰撞检查的时刻
    bool collides_ahead_ = false;
    ego_planner::QuadrotorCommand last_cmd_;
    bool have_cmd_ = false;

    Eigen::Vector3d last_pose_ = Eigen::Vector3d::Zero();     // 上一帧位姿位置
    Eigen::Vector3d last_recenter_ = Eigen::Vector3d::Zero(); // 上次滑动窗口时的位置
    bool have_pose_ = false;

    Stats stats_;
    double newest_pose_time_ = -1e9; // 最新位姿时间（判断位姿是否跟上）
    double last_scan_time_ = -1e9;   // 最近一帧点云时间
  };

} // namespace ego_onboard

#endif
