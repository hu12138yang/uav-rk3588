#include "onboard/offboard_pilot.h"
#include "onboard/route_utils.h"
#include <cmath>
#include <algorithm>
#include <string>

namespace ego_onboard
{

  const char *phaseName(Phase p)
  {
    switch (p)
    {
    case Phase::IDLE: return "IDLE";
    case Phase::LINK: return "LINK";
    case Phase::WAIT_GROUND_START: return "WAIT_GROUND_START";
    case Phase::PRE_OFFBOARD: return "PRE_OFFBOARD";
    case Phase::OFFBOARD_CRUISE: return "OFFBOARD_CRUISE";
    case Phase::HOLD: return "HOLD";
    case Phase::FAILED: return "FAILED";
    }
    return "?";
  }

  const char *holdReasonName(HoldReason r)
  {
    switch (r)
    {
    case HoldReason::NONE: return "NONE";
    case HoldReason::GPS_LOST: return "GPS_LOST";
    case HoldReason::SENSOR_LOST: return "SENSOR_LOST";
    case HoldReason::MISSION_DONE: return "MISSION_DONE";
    }
    return "?";
  }

  void OffboardPilot::fail(const std::string &why)
  {
    err_ = why;
    phase_ = Phase::FAILED;
  }

  // ---------------------------------------------------------------- 启动
  void OffboardPilot::start(const std::vector<GeoWaypoint> &route)
  {
    route_ = route;
    if (route_.size() < 2 || route_.size() > 65533 ||
        !std::isfinite(cfg_.cruise_alt_m) || cfg_.cruise_alt_m <= 0.5 ||
        !std::isfinite(cfg_.handover_alt_tolerance_m) || cfg_.handover_alt_tolerance_m <= 0)
    {
      fail("invalid route size / cruise handover height");
      return;
    }
    for (const auto &w : route_)
      if (!std::isfinite(w.lat_deg) || !std::isfinite(w.lon_deg) ||
          !std::isfinite(w.alt_rel_m) || std::abs(w.lat_deg) > 90 ||
          std::abs(w.lon_deg) > 180 || w.alt_rel_m <= 0.5)
      {
        fail("invalid route coordinate / height");
        return;
      }
    bool added = false;
    std::string why;
    if (!ensureReturnToStart(route_, 65533, added, why))
    {
      fail(why);
      return;
    }
    leg_ = 0;
    planner_setpoints_ = cruise_hold_setpoints_ = 0;
    mode_switch_sent_ = false;
    waiting_offboard_ = false;
    err_.clear();
    phase_ = Phase::LINK;
    t_start_ = -1.0;
    t_phase_ = 0.0;

    // GPS 健康状态
    gps_fix_type_ = 0;
    gps_sats_ = 0;
    gps_valid_ = false;
    t_last_gps_ = -1e9;
    t_gps_bad_ = -1e9;
    t_gps_good_ = -1e9;
    // setpoint 的 type_mask：位置 + yaw 必发；速度前馈按配置
    {
      const uint16_t kIgnoreAcc = (uint16_t)((1 << 6) | (1 << 7) | (1 << 8));
      const uint16_t kIgnoreYawRate = (uint16_t)(1 << 11);
      const uint16_t kIgnoreVel = (uint16_t)((1 << 3) | (1 << 4) | (1 << 5));
      sp_cfg_.type_mask = (uint16_t)(kIgnoreAcc | kIgnoreYawRate |
                                     (cfg_.velocity_feedforward ? 0 : kIgnoreVel));
    }

    // 安全悬停状态
    hold_pos_.setZero();
    hold_reason_ = HoldReason::NONE;
    hold_count_ = 0;
    gps_drop_count_ = 0;
    sensing_drop_count_ = 0;
    t_hold_clear_ = -1e9;
    sensing_healthy_ = true;
    reckoner_.reset();
  }

  // ---------------------------------------------------------------- 接收
  void OffboardPilot::onMavlinkBytes(const uint8_t *data, size_t len)
  {
    MavMessage msg;
    for (size_t i = 0; i < len; ++i)
    {
      if (!parser_.push(data[i], msg))
        continue;
      if (!msg.crc_checked || msg.sysid != cfg_.target_system || msg.compid != cfg_.target_component) continue;
      switch (msg.msgid)
      {
      case Timesync::ID: {
        Timesync m;
        if (Timesync::decode(msg.payload.data(), msg.payload.size(), m) &&
            (!m.target_system || m.target_system == cfg_.sysid) &&
            (!m.target_component || m.target_component == cfg_.compid)) {
          if (m.tc1 > 0) alignment_.accept(m.tc1, m.ts1, t_now_);
          else if (m.tc1 == 0) {
            m.tc1 = int64_t(t_now_ * 1e9); m.target_system = cfg_.target_system;
            m.target_component = cfg_.target_component; send(m, outbox_);
          }
        }
        break;
      }
      case SystemTime::ID: {
        SystemTime m;
        if (SystemTime::decode(msg.payload.data(), msg.payload.size(), m)) fcu_unix_ = m.time_unix_usec * 1e-6;
        break;
      }
      case Heartbeat::ID:
      {
        Heartbeat hb;
        if (Heartbeat::decode(msg.payload.data(), msg.payload.size(), hb))
          onHeartbeat(hb, t_now_);
        break;
      }
      case HomePosition::ID:
      {
        HomePosition hp;
        if (HomePosition::decode(msg.payload.data(), msg.payload.size(), hp))
          onHomePosition(hp);
        break;
      }
      case GlobalPositionInt::ID:
      {
        GlobalPositionInt g;
        if (GlobalPositionInt::decode(msg.payload.data(), msg.payload.size(), g))
          onGlobalPositionInt(g);
        break;
      }
      case Attitude::ID:
      {
        Attitude a;
        if (Attitude::decode(msg.payload.data(), msg.payload.size(), a))
          onAttitude(a);
        break;
      }
      case GpsRawInt::ID:
      {
        GpsRawInt g;
        if (GpsRawInt::decode(msg.payload.data(), msg.payload.size(), g))
          onGpsRawInt(g);
        break;
      }
      case CommandAck::ID:
      {
        CommandAck ak;
        if (CommandAck::decode(msg.payload.data(), msg.payload.size(), ak))
          onCommandAck(ak);
        break;
      }
      default:
        break;
      }
    }
  }

  void OffboardPilot::onHeartbeat(const Heartbeat &hb, double now)
  {
    // 只认飞控（非 GCS 类型）的心跳
    if (hb.type == 6)
      return;
    armed_ = hb.armed();
    main_mode_ = hb.mainMode();
    sub_mode_ = hb.subMode();
    t_last_fc_hb_ = now;
  }

  void OffboardPilot::onHomePosition(const HomePosition &hp)
  {
    if (std::abs(hp.latitude * 1e-7)>90 || std::abs(hp.longitude * 1e-7)>180) return;
    if (origin_.valid && (std::abs(origin_.lat_deg-hp.latitude*1e-7)>1e-7 || std::abs(origin_.lon_deg-hp.longitude*1e-7)>1e-7)) {
      fail("HOME changed: restart with a consistent map origin"); return;
    }
    origin_.lat_deg = hp.latitude * 1e-7;
    origin_.lon_deg = hp.longitude * 1e-7;
    origin_.valid = true;
    pose_asm_.setOrigin(origin_);
  }

  void OffboardPilot::onGlobalPositionInt(const GlobalPositionInt &g)
  {
    if (!origin_.valid)
      return;
    const double boot = g.time_boot_ms * 0.001;
    if (last_pos_boot_ >= 0 && boot < last_pos_boot_ - 0.05) { alignment_.reset(); have_pos_ = false; fail("FCU boot clock reset"); return; }
    if (boot <= last_pos_boot_) return;
    last_pos_boot_ = boot; t_last_pos_ = t_now_;
    if (std::abs(g.lat * 1e-7)>90 || std::abs(g.lon * 1e-7)>180) { have_pos_=false; return; }
    pose_asm_.onGlobalPositionInt(g);
    pos_enu_ = pose_asm_.positionEnu();
    have_pos_ = true;

    // 航位推算：定位有效就用它重置锚点；无效则对速度做一次积分（输出经纬高给上报用）
    if (gps_fix_type_ >= cfg_.gps_min_fix && t_now_-t_last_gps_<=cfg_.gps_rx_timeout_s)
    {
      GeoFix f;
      f.lat_deg = g.lat * 1e-7;
      f.lon_deg = g.lon * 1e-7;
      f.alt_m = g.relative_alt * 1e-3;
      reckoner_.onValidFix(f, t_now_);
    }
    else
    {
      reckoner_.onVelocity(pose_asm_.velocityEnu(), t_now_);
    }
  }

  void OffboardPilot::onGpsRawInt(const GpsRawInt &g)
  {
    gps_fix_type_ = g.fix_type;
    gps_sats_ = g.satellites_visible;
    t_last_gps_ = t_now_;
  }

  void OffboardPilot::onAttitude(const Attitude &a)
  {
    if (!std::isfinite(a.roll) || !std::isfinite(a.pitch) || !std::isfinite(a.yaw)) return;
    const double boot = a.time_boot_ms * 0.001;
    if (last_att_boot_ >= 0 && boot < last_att_boot_ - 0.05) { alignment_.reset(); fail("FCU attitude clock reset"); return; }
    if (boot <= last_att_boot_) return;
    last_att_boot_ = boot; t_last_att_ = t_now_;
    if (!alignment_.valid(t_now_) || std::abs(alignment_.hostTime(boot) - t_now_) > cfg_.pose_timeout_s) return;
    if (t_now_ - t_last_pos_ > cfg_.position_extrapolation_s) return;
    ego_sensor::TimedPose tp;
    if (!pose_asm_.onAttitude(a, boot, tp)) return;
    tp.t=alignment_.hostTime(boot);
    // 位姿和定位锚点必须使用同一个时钟映射，逐点查询才可继续按速度外推。
    tp.position_anchor.t += tp.t - boot;
    if (t_now_-tp.t>cfg_.pose_timeout_s || tp.t>t_now_+0.005) return;
    hold_yaw_ = std::atan2(tp.q.toRotationMatrix()(1,0),tp.q.toRotationMatrix()(0,0));
    pos_ctrl_=tp.p; latest_pose_=tp;
    if(planner_)planner_->onPose(tp);
  }

  void OffboardPilot::onCommandAck(const CommandAck &ack)
  {
    // Only Offboard mode switching is owned by this controller. Ignore ACKs
    // for arm/takeoff/landing commands sent by the independent ground station.
    (void)ack;
  }

  void OffboardPilot::sendHeartbeat(std::vector<std::vector<uint8_t>> &out, double now)
  {
    Heartbeat hb; // 默认即 GCS 心跳：type=6 autopilot=8 base_mode=192 status=4
    send(hb, out);
    t_last_hb_ = now;
  }

  // ---------------------------------------------------------------- 周期
  void OffboardPilot::tick(double now, std::vector<std::vector<uint8_t>> &out)
  {
    t_now_ = now;
    if (t_start_ < 0)
      t_start_ = now;

    // 先吐出接收路径上攒下的帧
    for (auto &f : outbox_)
      out.push_back(std::move(f));
    outbox_.clear();

    // 心跳
    if (now - t_last_hb_ >= 1.0 / cfg_.heartbeat_hz)
      sendHeartbeat(out, now);

    if (now - t_last_sync_request_ >= 0.2) {
      Timesync m; m.ts1 = alignment_.request(now); m.target_system = cfg_.target_system;
      m.target_component = cfg_.target_component; send(m, out); t_last_sync_request_ = now;
    }
    if (!origin_.valid && now - t_last_home_request_ >= 2.0) {
      CommandLong request; request.command=512; request.target_system=cfg_.target_system;
      request.target_component=cfg_.target_component; request.param1=242;
      send(request,out); t_last_home_request_=now;
    }
    if (cfg_.observe_only) {
      if (cfg_.request_rates && !rates_requested_ && now - t_last_fc_hb_ < cfg_.link_timeout) {
        rates_requested_ = true;
        send(RequestDataStream::forMessage(GlobalPositionInt::ID, cfg_.rate_global_position_hz, cfg_.target_system), out);
        send(RequestDataStream::forMessage(Attitude::ID, cfg_.rate_attitude_hz, cfg_.target_system), out);
      }
      return; // NO mission upload, mode change, arming, or setpoint in observe mode.
    }
    const bool pose_ok = poseFresh(now);
    const bool timing_ok = clockReady(now) && input_ready_;
    const bool active = phase_ == Phase::OFFBOARD_CRUISE || phase_ == Phase::HOLD;
    if (active && !clockReady(now)) { fail("FCU clock synchronization lost"); return; }
    if (active && !pose_ok) { fail("position/attitude stream stale"); return; }

    // ---- GPS 健康判定 ----
    // 判据只有两个：GPS_RAW_INT 还在来（2Hz）、且 fix_type 够好。
    // 位置本身不在这里重算 —— 飞控 EKF 已经在融合 IMU，丢星后它会继续输出推算位置，
    // 我们只是消费 GLOBAL_POSITION_INT，并在定位不可信时停下来等。
    const bool gps_rx = (t_last_gps_ > -1e8) && (now - t_last_gps_ <= cfg_.gps_rx_timeout_s);
    const bool gps_ok = !cfg_.gps_failsafe || (gps_rx && gps_fix_type_ >= cfg_.gps_min_fix);
    if (gps_ok)
    {
      gps_valid_ = true;
      if (t_gps_good_ < -1e8)
        t_gps_good_ = now; // 连续有效的起始时刻
      t_gps_bad_ = -1e9;
    }
    else
    {
      gps_valid_ = false;
      if (t_gps_bad_ < -1e8)
        t_gps_bad_ = now; // 连续无效的起始时刻
      t_gps_good_ = -1e9;
    }

    // ---- 感知健康判定 ----
    // 连续丢帧 = 点云没进图，地图里全是"未知"，EGO 会规划出一条直线。
    // 这种失效在界面上看不出来，所以必须主动停下来。
    sensing_healthy_ = !cfg_.sensing_failsafe || !planner_ || planner_->sensingHealthy();

    // ---- 统一恢复判据：所有异常都清除了，并且稳定一段时间 ----
    const bool all_clear = gps_ok && sensing_healthy_ && timing_ok && pose_ok;
    if (all_clear)
    {
      if (t_hold_clear_ < -1e8)
        t_hold_clear_ = now;
    }
    else
    {
      t_hold_clear_ = -1e9;
    }

    if (phase_ == Phase::IDLE || phase_ == Phase::FAILED)
      return;

    if ((phase_ == Phase::PRE_OFFBOARD || active) && now - t_last_setpoint_ < 1.0 / cfg_.setpoint_hz) return;
    switch (phase_)
    {
    // ---------------------------------------------------------- S0 链路
    case Phase::LINK:
    {
      const bool fc_alive = (t_last_fc_hb_ > -1e8) && (now - t_last_fc_hb_ < cfg_.link_timeout);
      // 一看到飞控就先提频：默认 2Hz 位置 / 10Hz 姿态不够用（协议 2.5.1）
      if (fc_alive && cfg_.request_rates && !rates_requested_)
      {
        rates_requested_ = true;
        send(RequestDataStream::forMessage(GlobalPositionInt::ID, cfg_.rate_global_position_hz, cfg_.target_system), out);
        send(RequestDataStream::forMessage(Attitude::ID, cfg_.rate_attitude_hz, cfg_.target_system), out);
        send(RequestDataStream::forMessage(245, cfg_.rate_extension_hz, cfg_.target_system), out);
      }
      if (!fc_alive || !origin_.valid || !have_pos_)
      {
        if (now - t_start_ > 30.0)
          fail("link/home/position timeout");
        break;
      }
      // 参考航线（ENU，原点 = home）
      route_enu_.clear();
      for (const auto &w : route_)
      {
        const Eigen::Vector2d xy = geoToEnuXY(w.lat_deg, w.lon_deg, origin_);
        route_enu_.push_back(Eigen::Vector3d(xy.x(), xy.y(), w.alt_rel_m));
      }
      if (planner_)
      {
        planner_->setReferenceRoute(route_enu_);
        if (route_enu_.size() >= 2)
          planner_->setLeg(route_enu_[0], route_enu_[1], route_enu_.size() == 2);
      }
      // Route and waypoint progress stay onboard; the FCU mission protocol is not used.
      phase_ = Phase::WAIT_GROUND_START;
      t_phase_ = now;
      break;
    }

    // GCS starts flight; onboard only observes and waits for stable POS_HOLD.
    case Phase::WAIT_GROUND_START:
    {
      const bool fc_alive = now - t_last_fc_hb_ < cfg_.link_timeout;
      if (fc_alive && armed_ && pose_ok && timing_ok && sensing_healthy_ && gps_ok && main_mode_ == MODE_POS_HOLD &&
          !route_enu_.empty() && (pos_enu_.head<2>()-route_enu_.front().head<2>()).norm() <= cfg_.handover_position_tolerance_m &&
          std::abs(pos_enu_.z() - cfg_.cruise_alt_m) <= cfg_.handover_alt_tolerance_m)
      {
        phase_ = Phase::PRE_OFFBOARD;
        t_phase_ = now;
        mode_switch_sent_ = false;
      }
      break;
    }

    // ------------------------------- S4 先连发 setpoint，再切 Offboard
    case Phase::PRE_OFFBOARD:
    {
      if (!pose_ok || !timing_ok || !sensing_healthy_ || !gps_ok || !armed_ || (main_mode_ != MODE_POS_HOLD && main_mode_ != MODE_OFFBOARD))
      {
        // Do not override GCS landing / RTL / disarm during handover.
        phase_ = Phase::WAIT_GROUND_START;
        mode_switch_sent_ = false;
        waiting_offboard_ = false;
        break;
      }
      // 从当前位姿起步（速度/加速度为 0），避免切模式瞬间位置跳变
      ego_planner::QuadrotorCommand cmd;
      cmd.position = pos_enu_;
      cmd.velocity.setZero();
      cmd.acceleration.setZero();
      cmd.yaw = hold_yaw_;
      cmd.yaw_dot = 0.0;
      if (!cmd.position.allFinite() || !cmd.velocity.allFinite() || !cmd.acceleration.allFinite() ||
          !std::isfinite(cmd.yaw) || !std::isfinite(cmd.yaw_dot)) { fail("nonfinite control target"); break; }
      SetPositionTargetLocalNed sp;
      toSetpoint(cmd, alignment_.bootMs(now), sp_cfg_, sp);
      sp.target_system = cfg_.target_system;
      sp.target_component = cfg_.target_component;
      send(sp, out);
      t_last_setpoint_ = now;

      if (!mode_switch_sent_ && now - t_phase_ >= cfg_.pre_offboard_stream_s)
      {
        send(CommandLong::setMode(MODE_OFFBOARD, 0, cfg_.target_system), out);
        t_cmd_sent_ = now;
        last_cmd_ = CMD_DO_SET_MODE;
        mode_switch_sent_ = true;
        waiting_offboard_ = true;
      }
      if (waiting_offboard_ && main_mode_ == MODE_OFFBOARD)
      {
        waiting_offboard_ = false;
        phase_ = Phase::OFFBOARD_CRUISE;
        t_phase_ = now;
      }
      else if (mode_switch_sent_ && now - t_cmd_sent_ > 3.0)
        fail("enter offboard timeout");
      break;
    }

    // --------------------------- S5 航点间 Offboard（我们的系统接管）
    case Phase::OFFBOARD_CRUISE:
    {
      // 飞控失联：此时没法悬停，只能报错（不能一直往断掉的链路写 setpoint）
      const bool fc_alive = (t_last_fc_hb_ > -1e8) && (now - t_last_fc_hb_ < cfg_.link_timeout);
      if (!fc_alive)
      {
        fail("fcu link lost");
        break;
      }
      if (main_mode_ != MODE_OFFBOARD)
      {
        fail("dropped out of offboard");
        break;
      }

      // ---- 飞行任务异常，统一归到安全悬停 ----
      // 判据只分两类：定位不可信、感知不可信。
      // 规划失败不在这里 —— 规划层失败时本来就发悬停指令并持续重试，切状态反而停掉重试。
      // 悬停期间不规划、不避障；位置/姿态估计继续，并通过 UDP 上报。
      HoldReason why = HoldReason::NONE;
      if (!gps_ok && now - t_gps_bad_ >= cfg_.gps_loss_confirm_s)
        why = HoldReason::GPS_LOST;
      else if (!sensing_healthy_ || !timing_ok)
        why = HoldReason::SENSOR_LOST;

      if (why != HoldReason::NONE)
      {
        hold_pos_ = pos_ctrl_; // 以当前位姿为悬停点
        hold_reason_ = why;
        ++hold_count_;
        if (why == HoldReason::GPS_LOST)
          ++gps_drop_count_;
        else
          ++sensing_drop_count_;
        t_hold_clear_ = -1e9; // 恢复计时从进入悬停重新开始，不能沿用之前的
        phase_ = Phase::HOLD;
        t_phase_ = now;
        break;
      }

      ego_planner::QuadrotorCommand cmd;
      bool ok = false;
      if (cfg_.use_planner && planner_)
        ok = planner_->step(now, pos_ctrl_, cmd);
      if (!ok)
      {
        // 规划失败 → 原地悬停（实机这里应叠加飞控侧急停/悬停）
        cmd.position = pos_enu_;
        cmd.velocity.setZero();
        cmd.acceleration.setZero();
        cmd.yaw = hold_yaw_;
        cmd.yaw_dot = 0.0;
      }
      if (!cmd.position.allFinite() || !cmd.velocity.allFinite() || !cmd.acceleration.allFinite() ||
          !std::isfinite(cmd.yaw) || !std::isfinite(cmd.yaw_dot)) { fail("nonfinite control target"); break; }
      SetPositionTargetLocalNed sp;
      toSetpoint(cmd, alignment_.bootMs(now), sp_cfg_, sp);
      sp.target_system = cfg_.target_system;
      sp.target_component = cfg_.target_component;
      send(sp, out);
      t_last_setpoint_ = now;

      // 区分 EGO 轨迹指令与巡航阶段的保底悬停，供运行日志核对输出来源。
      if (ok && cmd.trajectory_flag == ego_planner::TRAJECTORY_STATUS_READY)
        ++planner_setpoints_;
      else
        ++cruise_hold_setpoints_;

      // 航点到达由本地位置判定；推进本地航段并更新规划目标。
      if (leg_ + 1 < (int)route_enu_.size() && have_pos_)
      {
        if ((pos_ctrl_ - route_enu_[leg_ + 1]).norm() < cfg_.wp_reach_tol)
        {
          leg_++;
          if (leg_ + 1 < (int)route_enu_.size())
          {
            if (planner_)
              planner_->setLeg(route_enu_[leg_], route_enu_[leg_ + 1], leg_ + 2 == (int)route_enu_.size());
          }
          else
          {
            // 最后航点已返回 KML 起点：保持巡航高度，等地面站降落。
            hold_pos_ = pos_ctrl_;
            hold_reason_ = HoldReason::MISSION_DONE;
            ++hold_count_;
            t_hold_clear_ = -1e9;
            phase_ = Phase::HOLD;
            t_phase_ = now;
          }
        }
      }
      break;
    }

    // ----------------- S5b 安全悬停（任何飞行任务异常，仍在 Offboard 内）
    case Phase::HOLD:
    {
      // 这三类异常没法靠悬停解决，只能报错
      const bool fc_alive = (t_last_fc_hb_ > -1e8) && (now - t_last_fc_hb_ < cfg_.link_timeout);
      if (!fc_alive)
      {
        fail("fcu link lost during hold");
        break;
      }
      if (main_mode_ != MODE_OFFBOARD)
      {
        fail("dropped out of offboard during hold");
        break;
      }
      if (!have_pos_)
      {
        fail("no position estimate during hold");
        break;
      }

      // 原地悬停：冻结进入悬停那一刻的位置和航向。
      // 这里**不做任何规划** —— 数据已经不可信，继续避障等于盲飞。
      // 位置/姿态估计仍在跑（航位推算在 onGlobalPositionInt 里更新），由上层上报。
      ego_planner::QuadrotorCommand cmd;
      cmd.position = hold_pos_;
      cmd.velocity.setZero();
      cmd.acceleration.setZero();
      cmd.yaw = hold_yaw_;
      cmd.yaw_dot = 0.0;
      if (!cmd.position.allFinite() || !cmd.velocity.allFinite() || !cmd.acceleration.allFinite() ||
          !std::isfinite(cmd.yaw) || !std::isfinite(cmd.yaw_dot)) { fail("nonfinite control target"); break; }
      SetPositionTargetLocalNed sp;
      toSetpoint(cmd, alignment_.bootMs(now), sp_cfg_, sp);
      sp.target_system = cfg_.target_system;
      sp.target_component = cfg_.target_component;
      send(sp, out);
      t_last_setpoint_ = now;

      // 异常全部清除并稳定 -> 丢掉过期轨迹，回到巡航。
      // 任务完成（MISSION_DONE）不在此列：那是有意停机，等人工接管。
      if (hold_reason_ != HoldReason::MISSION_DONE && all_clear &&
          now - t_hold_clear_ >= cfg_.hold_recover_confirm_s)
      {
        if (planner_)
          planner_->dropTrajectory();
        hold_reason_ = HoldReason::NONE;
        phase_ = Phase::OFFBOARD_CRUISE;
        t_phase_ = now;
      }
      break;
    }

    default:
      break;
    }
  }

} // namespace ego_onboard
