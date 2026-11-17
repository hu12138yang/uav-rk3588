#ifndef ONBOARD_TIME_ALIGNMENT_H
#define ONBOARD_TIME_ALIGNMENT_H
// 时间对齐工具，只负责"把别人的时间映射到本机时间轴"，不负责校时：
//   TimeAlignment : 用 TIMESYNC 往返估计飞控时钟与本机单调时钟的固定偏差。
//   EpochBridge   : 把 PTP/UTC（Unix epoch）映射到本机单调时钟，并检测时钟跳变。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>

namespace ego_onboard {
// 本机单调时钟（秒），不受 NTP/手动改时间影响，所有内部计时都用它
inline double monotonicSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
// 本机 UTC 时间（Unix epoch 秒），只在跨系统对时（PTP/日志）时使用
inline double unixSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
// TIMESYNC mid-point estimator. Receive timestamps are NEVER used as sample times.
// RTT/2 is an estimated uncertainty bound, assuming a responsive peer clock.
// 偏差 = 本机发送/接收时刻的中点 - 飞控回带的时间戳；样本做 0.9/0.1 低通，
// 残差超限即认为发生跳变，清空后重新累积。
class TimeAlignment {
 public:
  // 样本采纳门限：往返时延上限、与当前估计的最大残差、样本有效期（秒）
  double max_rtt_s = 0.010, max_residual_s = 0.005, timeout_s = 3.0;
  int min_samples = 10;  // 至少累积这么多样本，valid() 才为真
  // 发起一次 TIMESYNC 请求，返回本次请求标识；对端必须原样带回
  int64_t request(double now) {
    pending_ = int64_t(now * 1e9); sent_ = now; return pending_;
  }
  // 处理应答：校验请求标识、RTT、残差；通过则更新偏差估计并返回 true
  bool accept(int64_t tc1, int64_t ts1, double now) {
    if (!pending_ || ts1 != pending_ || tc1 <= 0) { ++rejected_; return false; }
    pending_ = 0;
    const double rtt = now - sent_;
    if (rtt < 0 || rtt > max_rtt_s) { ++rejected_; return false; }
    // 长时间没有样本（链路中断过）先清空，避免用旧估计拼接新样本
    if (samples_ && now-last_ > timeout_s) reset();
    const double measured = (sent_ + now) * 0.5 - double(tc1) * 1e-9;
    if (samples_ && std::abs(measured - offset_) > max_residual_s) {
      reset(); ++rejected_; return false;
    }
    residual_ = samples_ ? std::abs(measured - offset_) : 0;
    offset_ = samples_ ? offset_ * 0.9 + measured * 0.1 : measured;
    rtt_ = rtt; last_ = now; ++samples_; return true;
  }
  // 时钟可用 = 样本足够 && 最近一次样本没有超时
  bool valid(double now) const { return samples_ >= min_samples && now >= last_ && now - last_ <= timeout_s; }
  // 飞控 time_boot_ms（秒） -> 本机单调时钟
  double hostTime(double boot_seconds) const { return boot_seconds + offset_; }
  // 本机单调时钟 -> 飞控 time_boot_ms（用于发给飞控的时间字段）
  uint32_t bootMs(double now) const { return uint32_t(uint64_t(std::llround(std::max(0.0, (now - offset_) * 1000)))); }
  double offset() const { return offset_; }
  double rtt() const { return rtt_; }
  double residual() const { return residual_; }
  int samples() const { return samples_; }
  uint64_t rejected() const { return rejected_; }
  uint64_t generation() const { return generation_; }
  // 清空全部样本（时钟跳变/链路重建后调用），generation 自增通知使用方失效
  void reset() { samples_ = 0; pending_ = 0; last_ = -1e9; ++generation_; }
 private:
  int64_t pending_ = 0;  // 尚未应答的请求标识，0 表示没有在途请求
  double sent_ = 0, offset_ = 0, last_ = -1e9, rtt_ = 0, residual_ = 0;
  int samples_ = 0;
  uint64_t rejected_ = 0, generation_ = 0;
};
// PTP master is CLOCK_REALTIME with ptpTimescale=0. Convert its Unix epoch
// to the host monotonic axis. Detect NTP/manual clock steps; reject and flush.
// 每帧采一对 (单调时钟, Unix 时间) 求偏移；偏移突变 > 5 ms 认为系统时间被
// NTP/人工改过，返回 false，调用方应丢弃该帧并清空相关状态。
class EpochBridge {
 public:
  // 采样一次，返回本机时钟是否稳定（false = 检测到跳变）
  bool sample(double mono, double epoch) {
    const double next = epoch - mono;
    const bool stable = !have_ || std::abs(next - offset_) <= 0.005;
    offset_ = next; have_ = true; return stable;
  }
  // Unix epoch 时间 -> 本机单调时钟
  double toMonotonic(double epoch) const { return epoch - offset_; }
 private:
  bool have_ = false;   // 是否已经有第一次采样
  double offset_ = 0;   // epoch - mono 的偏移
};
}
#endif
