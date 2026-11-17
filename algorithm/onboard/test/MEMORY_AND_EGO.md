# 内存占用与 EGO 输出验证

2026-11-16，在 Linux x86_64、Release 构建下测量，参数来自
`algorithm/onboard/config/onboard.yaml`。MiB = 1024² 字节，以下均为进程
驻留内存 RSS，不是虚拟地址空间，也不包含操作系统和其他服务的占用。

## 实测内存

| 场景 | 峰值 RSS |
| --- | ---: |
| 同一程序使用原 5 m/s 配置启动，66×66 m 地图，无雷达流 | 80.6 MiB |
| 当前 20 m/s 配置启动，216×216 m 地图，无雷达流 | 131.9 MiB |
| 当前配置，真实异步规划及飞控输出代码，每帧 2 万合成点连续输入 | 132.7 MiB |

启动测试使用真正的 `onboard_main`，串口通过 PTY 回环；点云负载测试使用
`ego_output_selftest`，包含真正的 `AsyncPlanner`、EGO、轨迹跟踪器和
`OffboardPilot`，不打开硬件。每帧 2 万点、约 10 Hz；这次输入了 256 万点，
测试期间 RSS 大致稳定在 130–134 MiB。记录包的 SDK 解码、组帧、去畸变及
异步入图另由 `lidar_system_replay_selftest` 覆盖。

这些结果不是实机整场飞行的内存上限。点云分布、队列积压、SDK 网络/日志
缓冲、分配器和 CPU 架构会影响结果。部署时可先给板载进程预留
256–512 MiB，操作系统及其他服务另算，再用实机日志校核。

主要内存来自固定地图和 A* 节点池：

- 栅格为 `864×864×79`，每格 1 字节，共 **56.24 MiB**。原地图为 5.25 MiB，
  此次扩图增加约 51 MiB。
- A* 池为 `100³` 个节点。本机 `sizeof(GridNode)=48`，节点有效数据约
  **45.78 MiB**，指针数组约 **7.71 MiB**，另有逐节点分配器开销。
- 本机 `sizeof(LidarPoint)=40`，每帧 2 万点约 **0.76 MiB**，尚不含容量余量、
  队列副本、去畸变点云和膨胀临时数组。

地图滚动移动固定缓冲；点云队列和位姿历史有长度限制，累计处理点数不代表
保留了所有历史点云。同时修复了 A* 重初始化丢失旧节点池、析构遗漏多级
指针数组的问题。`astar_lifetime_selftest` 连续重建 24 次并检查堆内存释放。

## EGO 确实参与输出的证据

`ego_output_selftest` 的首个局部目标在 `(70,0,50)`，畅通；墙位于中途
`x=25`，阻挡直线初始化轨迹。因此，该场景需要 EGO 的障碍搜索和样条优化，
不能仅靠适配器把目标点抬高通过。

测试使用真实控制状态机走过定位、时钟、感知门限及 Offboard 接管。一次完整
测试记录如下，具体次数会随线程调度略有变化：

```text
success=63 fail=0 optimizations=97 astar=14/14
matched=563 mismatched=0 deviation=5.64m rss_peak=132.7MiB
```

- 63 次规划成功，97 次优化调用，14 次 A* 搜索成功。
- 独立解析带 CRC 的最终 MAVLink `SET_POSITION_TARGET_LOCAL_NED`，563 条
  指令的位置和 yaw 与 EGO 输出经 ENU/NED 转换后的结果一致。
- 核对生产代码的 EGO 输出计数与独立解析计数相等，排除只预规划、未采用结果。
- 输出轨迹绕开膨胀墙体、最大偏离参考直线 5.64 m，并越过墙后到达 x>45 m。
- 当前仍采用位置+yaw 输出（`type_mask=0x09F8`）；速度字段被忽略不意味着
  EGO 被绕过，飞控跟踪的是 EGO 样条采样生成的位置。

合成飞控按收到的设定点作理想位置跟踪；这验证软件输出来源及避障行为，
不验证实际飞机动力学、真实 PTP 精度、雷达探测距离或实机跟踪误差。
原有 350 m 高速测试另验证了约 20 m/s 的轨迹、绕障和终点到达。

## 实机查看

主程序每秒新增一行 `[EGO]`：

- `plans_ok`、`optimizations`：成功规划数、实际优化调用数。
- `astar_ok=成功数/调用数`：遇到需要搜索的障碍时增长；空旷环境可以一直为 0。
- `tracked`：工作线程从 EGO 样条跟踪器取得的指令数，观察模式预规划也可增长。
- `sent`：巡航中由 `OffboardPilot` 编码、交给发送链路的 EGO 设定点数。
  在 `OFFBOARD_CRUISE` 阶段持续增长，才能证明规划结果用于控制输出；
  它不代表飞控已经实际跟踪到该位置。
- `cruise_holds`：巡航中生成的非轨迹悬停设定点数。
- `plan_last`、`plan_max`：规划耗时；`rss_peak`：该进程启动以来的 RSS 峰值。

## 复现

在项目根目录执行：

```bash
cmake -S algorithm/onboard -B /tmp/uav-final-build -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/uav-final-build -j4
/usr/bin/time -v /tmp/uav-final-build/ego_output_selftest algorithm/onboard/config/onboard.yaml
/tmp/uav-final-build/ego_output_selftest algorithm/onboard/config/onboard.yaml --astar-lifetime
```

完整 CTest 在构建目录运行：

```bash
cd /tmp/uav-final-build
ctest --output-on-failure -j2
```

本轮共 **14/14** 项测试通过。
