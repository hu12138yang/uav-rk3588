#ifndef ONBOARD_LIVOX_SOURCE_H
#define ONBOARD_LIVOX_SOURCE_H
#include <deque>
#include <mutex>
#include <string>
#include <livox_lidar_api.h>
#include <sensor/frame_assembler.h>
namespace ego_onboard {
// Livox Mid-360S 数据源：封装 SDK2 回调，把原始 UDP 点云包交给 FrameAssembler
// 拼成整帧，再放进有界队列供主循环取用。
// 线程模型：SDK2 回调线程只做入队/统计，pop() 由主循环调用；内部用 mutex_ 保护。
class LivoxSource {
 public:
  // 解析/过滤参数，来自 YAML 的 lidar 段
  struct Config {
    bool allow_zero_crc = false;        // 实测该设备点云 CRC 恒为 0，视为"未提供"而非校验失败
    std::string expected_serial;        // 只接受该序列号的雷达，防止接错设备
    int point_data_type = 1;            // 1 = Cartesian High（96 点 × 14 字节 + 36 字节头）
    double frame_duration_s = 0.1;      // 约 100 ms/帧（10 Hz）
    double gap_factor = 2.0;            // 包间异常大缺口的判定系数，用于清理半帧
    double min_range_m = 0.1, max_range_m = 100.0; // 点云距离过滤
    size_t queue_capacity = 3;          // 已组装帧的队列上限，满了丢最旧
  };
  // 运行统计与同步状态（主循环日志/健康判断用）
  struct Stats {
    uint64_t packets=0, invalid=0, frames=0, queue_drops=0, crc_missing=0;
    uint8_t time_type=0;                // 雷达时间类型（PTP/内部时钟等）
    double last_rx=0;                   // 最近一次收到点云的本机时刻
    uint8_t sync_type=0;                // 1 = PTP 同步成功，其它值视为未同步
    uint64_t last_sync_ns=0;            // 最近一次时间同步上报值
    int64_t time_offset_ns=0;           // 雷达上报的时间偏移
    double state_rx=0;                  // 最近一次收到状态包的本机时刻
    std::string serial, ip;             // 实际连上的设备序列号/IP
  };
  ~LivoxSource();
  // 启动：config 为 SDK2 JSON 路径，ip 为目标雷达地址；
  // diagnose_zero_crc 只采集原始数据用于排查；settings 为空时用 Config 默认值。
  bool start(const std::string &config, const std::string &ip, std::string &error, bool diagnose_zero_crc=false, const Config *settings=nullptr);
  void stop();
  // 取一帧已组装的点云；队列空返回 false
  bool pop(ego_sensor::LidarFrame &frame);
  Stats stats() const;
 private:
  static void state(uint32_t, uint8_t, const char *, void *);   // 设备状态回调
  static void points(uint32_t, uint8_t, LivoxLidarEthernetPacket *, void *); // 点云包回调
  static void info(uint32_t, const LivoxLidarInfo *, void *);   // 设备信息回调
  static void dataType(livox_status, uint32_t, LivoxLidarAsyncControlResponse *, void *); // 查询点云格式
  static void mode(livox_status, uint32_t, LivoxLidarAsyncControlResponse *, void *);     // 设置工作模式
  mutable std::mutex mutex_;                       // 保护 queue_ 与 stats_
  ego_sensor::FrameAssembler assembler_;           // 按帧时长/缺口把包拼成帧
  std::deque<ego_sensor::LidarFrame> queue_;       // 待消费的完整帧
  std::string wanted_ip_;                          // 期望连接的雷达 IP
  uint32_t handle_ = 0;                            // SDK2 设备句柄
  bool selected_ = false, started_ = false;        // 是否已锁定设备/已启动
  Stats stats_;
  Config config_;
};
}
#endif
