#ifndef ONBOARD_ASYNC_PLANNER_H
#define ONBOARD_ASYNC_PLANNER_H
#include <onboard/ego_planner_adapter.h>
#include <onboard/time_alignment.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <thread>

namespace ego_onboard {
  
class AsyncPlanner : public PlannerIO {
 public:
  // 启动工作线程；cfg 为规划/建图参数，ref 为高度基准
  AsyncPlanner(const EgoPlannerAdapterConfig &cfg, double ref) : cfg_(cfg), ref_(ref) {
    worker_ = std::thread([this]{
      try { run(); }
      catch(const std::exception &e) {
        // 工作线程异常：标记不健康并清空命令，控制侧会因此进入安全保持
        std::lock_guard<std::mutex> lock(m_); healthy_=false; have_cmd_=false;
        std::fprintf(stderr,"[PLANNER] worker stopped: %s\n",e.what());
      }
    });
  }
  ~AsyncPlanner() { running_ = false; wake_.notify_all(); worker_.join(); }
  // 切换航段：作废旧命令并递增 generation，防止旧航段算出的结果被采用
  void setLeg(const Eigen::Vector3d &a, const Eigen::Vector3d &b, bool last) override {
    std::lock_guard<std::mutex> l(m_); a_=a; b_=b; last_=last; leg_=true; have_cmd_=false; ++generation_; wake_.notify_one();
  }
  // 飞控位姿入队；最多保留 500 帧历史（供点云逐点去畸变按时间查找）
  void onPose(const ego_sensor::TimedPose &p) override {
    std::lock_guard<std::mutex> l(m_); poses_.push_back(p); latest_=p;
    if (poses_.size() > 500) poses_.pop_front(); wake_.notify_one();
  }
  // 原始雷达帧入队；只保留最近 3 帧，避免规划跟不上时无界堆积
  void onScanFrame(const ego_sensor::LidarFrame &f) override {
    std::lock_guard<std::mutex> l(m_);
    if (scans_.size() >= 3) scans_.pop_front(); scans_.push_back(f); wake_.notify_one();
  }
  // 控制侧每拍调用：命令在 0.15 s 内有效则直接复用；
  // 否则返回 false，并用当前位置填充一个零速度的保底指令。
  bool step(double now, const Eigen::Vector3d &p, ego_planner::QuadrotorCommand &cmd) override {
    std::lock_guard<std::mutex> l(m_); requested_=now; wake_.notify_one();
    if (have_cmd_ && now - command_time_ <= 0.15) { cmd=command_; return true; }
    cmd.position=p; cmd.velocity.setZero(); cmd.acceleration.setZero(); cmd.yaw=0; cmd.yaw_dot=0;
    return false;
  }
  // 丢弃当前轨迹（丢星悬停恢复、换任务时用）；同时作废在途命令
  void dropTrajectory() override {
    std::lock_guard<std::mutex> l(m_); drop_=true; have_cmd_=false; requested_=-1e9; ++generation_;
  }
  // 时钟不连续：清空位姿历史与在途帧并丢弃轨迹，但保留世界系栅格地图
  void resetClockState() override {
    std::lock_guard<std::mutex> l(m_);
    poses_.clear(); scans_.clear(); have_cmd_=false; reset_clock_=true; ++generation_; wake_.notify_one();
  }
  // 感知健康 = 建图线程正常 && 地图在 sensing_timeout_s 内更新过 && 工作线程 0.25 s 内活过
  bool sensingHealthy() const override {
    std::lock_guard<std::mutex> l(m_);
    return healthy_ && monotonicSeconds() - last_map_ <= cfg_.sensing_timeout_s &&
           monotonicSeconds() - last_worker_ <= 0.25;
  }
  EgoPlannerAdapter::Stats stats() const { std::lock_guard<std::mutex> l(m_); return stats_; }
 private:
  // 工作线程主循环：交换队列 -> 转发航段/位姿 -> 按位姿覆盖顺序消费原始帧
  // -> 推进规划 -> 回写命令与统计（generation 变了就丢弃这一拍的结果）
  void run() {
    EgoPlannerAdapter adapter; adapter.init(cfg_); adapter.prepareMap(ref_);
    std::deque<ego_sensor::LidarFrame> pending;   // 等待位姿覆盖的原始帧
    while (running_) {
      std::deque<ego_sensor::TimedPose> poses;
      std::deque<ego_sensor::LidarFrame> scans;
      Eigen::Vector3d a,b; bool leg,drop,reset_clock,last; double request; ego_sensor::TimedPose latest;
      uint64_t generation;
      {
        std::unique_lock<std::mutex> l(m_);
        wake_.wait_for(l, std::chrono::milliseconds(20));  // 无事件也按 50 Hz 醒来
        poses.swap(poses_); scans.swap(scans_); a=a_; b=b_; last=last_; leg=leg_; leg_=false;
        drop=drop_; drop_=false; reset_clock=reset_clock_; reset_clock_=false;
        request=requested_; latest=latest_; generation=generation_;
      }
      if (leg) adapter.setLeg(a,b,last);
      if (drop) adapter.dropTrajectory();
      // 时钟不连续：清位姿缓冲，并丢弃这一批已按旧时间映射转换过的位姿与帧
      if (reset_clock) { pending.clear(); poses.clear(); adapter.clearPoseHistory(); }
      for (const auto &p:poses) adapter.onPose(p);
      for (auto &f:scans) pending.push_back(std::move(f));
      while (pending.size() > 3) pending.pop_front();
      const double now=monotonicSeconds();
      // 只把"扫描结束时刻已有位姿覆盖"的帧送进建图；过期帧直接丢弃
      while (!pending.empty()) {
        auto &f=pending.front();
        if (now - f.t_base > cfg_.sensing_timeout_s) { pending.pop_front(); continue; }
        if (!adapter.frameCovered(f)) break; // Wait for FCU pose bracket at scan end.
        adapter.onScanFrame(f); pending.pop_front();
      }
      ego_planner::QuadrotorCommand cmd;
      // 只有控制侧最近 0.1 s 内请求过、感知健康、且规划器给出轨迹时才产出命令
      const bool ok=now-request < 0.1 && adapter.sensingHealthy() && adapter.step(now,latest.p,cmd);
      {
        std::lock_guard<std::mutex> l(m_);
        const auto st=adapter.stats();
        if (st.inflate_samples > stats_.inflate_samples) last_map_=monotonicSeconds(); // 有新点进图
        stats_=st; healthy_=adapter.sensingHealthy(); last_worker_=monotonicSeconds();
        // 航段已切换（generation 变化）时，这一拍结果作废
        if (ok && generation==generation_) { command_=cmd; command_time_=now; have_cmd_=true; }
      }
    }
  }
  EgoPlannerAdapterConfig cfg_; double ref_;
  mutable std::mutex m_; std::condition_variable wake_; std::thread worker_; std::atomic<bool> running_{true};
  std::deque<ego_sensor::TimedPose> poses_;        // 待处理位姿（有界 500）
  std::deque<ego_sensor::LidarFrame> scans_;       // 待处理原始帧（有界 3）
  ego_sensor::TimedPose latest_;                   // 最近一帧位姿（规划起点）
  Eigen::Vector3d a_=Eigen::Vector3d::Zero(), b_=Eigen::Vector3d::Zero(); // 当前航段起点/终点
  bool last_=false, leg_=false, drop_=false, reset_clock_=false, healthy_=false, have_cmd_=false;
  double requested_=-1e9, command_time_=-1e9, last_map_=-1e9, last_worker_=-1e9;
  uint64_t generation_=0;                          // 航段代次，用于丢弃过期结果
  ego_planner::QuadrotorCommand command_; EgoPlannerAdapter::Stats stats_;
};
}
#endif
