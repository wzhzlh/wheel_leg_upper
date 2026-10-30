#ifndef LEG_CALC__CONTROL_STATE_HPP_
#define LEG_CALC__CONTROL_STATE_HPP_

// 控制状态机 + 安全监督（纯逻辑，不依赖 ROS，便于单测）
//
// 设计要点（对应“500 Hz 控制环独立运行、上层命令只更新目标”的分层原则）：
//   * 状态机只在控制环的周期边界上推进，输入是本周期采到的健康快照；
//   * 故障不是“一律输出零力矩”，而是按类型映射到不同的安全动作：
//       - 通信短暂丢失 / 上层命令超时 → 降级：速度目标归零，仍保留平衡与 VMC
//       - IMU / 关节反馈丢失、姿态越界、急停、命令长期丢失 → 停机：
//         关节失力、轮子位置保持（把轮子当抱闸，避免直接撤掉平衡力矩导致翻倒）
//   * 停机后必须先把使能请求撤掉（enable=0）才允许回到 Idle，
//     避免故障消失后机器人“自己重新站起来”。
//   * 姿态越界只在真正出力（Run / Degraded）时算故障：失能时机器人是被动
//     躺倒 / 自由摆动的，此时报姿态故障既保护不了任何东西，又会让状态在
//     Idle ↔ Fault 之间反复跳动。失能期间姿态越界只用于“拒绝站起”。

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace leg_calc
{

/// 控制系统状态
enum class ControlState : int
{
  kInit = 0,      ///< 上电自检：传感器未就绪或未满足启动条件，输出安全
  kIdle = 1,      ///< 待命：关节失力、轮子保持，等待使能
  kRun = 2,       ///< 正常运行：平衡 + VMC + 速度跟踪
  kDegraded = 3,  ///< 降级：命令超时，速度目标归零，保留平衡与 VMC
  kFault = 4,     ///< 故障停机：关节失力、轮子位置保持，需重新使能
};

/// 状态名（日志 / 调试用）
inline const char * toString(ControlState state)
{
  switch (state) {
    case ControlState::kInit: return "INIT";
    case ControlState::kIdle: return "IDLE";
    case ControlState::kRun: return "RUN";
    case ControlState::kDegraded: return "DEGRADED";
    case ControlState::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

/// 故障位（可同时置位，便于上位机按位解读）
enum FaultBits : uint32_t
{
  kFaultNone = 0u,
  kFaultImu = 1u << 0,         ///< IMU 未就绪或数据超时
  kFaultJointState = 1u << 1,  ///< 关节反馈未就绪或超时
  kFaultCommand = 1u << 2,     ///< 上层命令长期丢失
  kFaultAttitude = 1u << 3,    ///< 姿态 / 角速度越界
  kFaultEstop = 1u << 4,       ///< 上层急停
  kFaultWheelState = 1u << 5,  ///< 轮反馈未就绪 / 超时 / 非有限值
  kFaultLqr = 1u << 6,         ///< 当前高度的反馈增益不可用
};

/// 会直接导致停机的故障位
/// （命令短时丢失只会置 command_fresh=false 进入降级，
///   只有超过 command_fault_timeout 才置 kFaultCommand 并停机）
constexpr uint32_t kHardFaultMask =
  kFaultImu | kFaultJointState | kFaultAttitude | kFaultEstop | kFaultCommand |
  kFaultWheelState | kFaultLqr;

/// 安全门限
struct SafetyLimits
{
  double imu_timeout{0.10};          ///< IMU 数据年龄上限 [s]
  double joint_timeout{0.10};        ///< 关节反馈年龄上限 [s]
  double wheel_timeout{0.10};        ///< 轮速反馈年龄上限 [s]（LQR 的 x/x_dot 全靠它）
  double command_timeout{0.30};      ///< 上层命令超时 → 降级 [s]
  double command_fault_timeout{1.00};///< 上层命令长期丢失 → 停机 [s]
  double max_pitch{0.60};            ///< |pitch| 上限 [rad]，约 34°
  double max_pitch_rate{6.00};       ///< |pitch_dot| 上限 [rad/s]
  int startup_cycles{250};           ///< 启动前需要连续健康的周期数（500 Hz 下 0.5 s）
};

/// 一个控制周期的健康快照
struct SafetyInputs
{
  bool imu_ok{false};
  double imu_age{1e9};
  bool joint_ok{false};
  double joint_age{1e9};
  bool wheel_ok{false};          ///< 轮速/轮角反馈是否有效（LQR 的 x、x_dot 来源）
  double wheel_age{1e9};         ///< 距最近一次有效轮反馈的时间 [s]
  bool lqr_ok{true};            ///< 当前高度的增益有效
  bool command_seen{false};      ///< 是否曾经收到过有效命令
  double command_age{1e9};       ///< 距最近一次有效命令的时间 [s]
  double pitch{0.0};
  double pitch_rate{0.0};
  bool estop{false};             ///< 上层急停请求
  bool enable_request{false};    ///< 上层使能请求
};

/// 状态机输出：本周期允许做什么
struct SafetyStatus
{
  ControlState state{ControlState::kInit};
  uint32_t faults{kFaultNone};
  bool sensors_ok{false};       ///< IMU + 关节反馈都新鲜
  bool balance_enabled{false};  ///< 是否允许输出平衡 / VMC 力矩
  bool joints_enabled{false};   ///< 是否允许输出关节力矩
  bool command_fresh{false};    ///< 命令是否在新鲜期内（决定是否跟踪速度目标）
};

/**
 * @brief 控制状态机
 *
 * 用法：每个控制周期调用一次 update()，用返回的 SafetyStatus 决定输出什么。
 * 该类不做任何 IO，所有判断都基于传入的快照，因此可以直接在单元测试里驱动。
 */
class SafetySupervisor
{
public:
  explicit SafetySupervisor(const SafetyLimits & limits = SafetyLimits())
  : limits_(limits) {}

  void setLimits(const SafetyLimits & limits) {limits_ = limits;}
  const SafetyLimits & limits() const {return limits_;}

  ControlState state() const {return state_;}
  uint32_t faults() const {return faults_;}

  /// 强迫回到初始状态（例如重新配置参数后）
  void reset()
  {
    state_ = ControlState::kInit;
    faults_ = kFaultNone;
    healthy_cycles_ = 0;
    locked_faults_ = kFaultNone;
  }

  SafetyStatus update(const SafetyInputs & in)
  {
    SafetyStatus out;
    const bool controlling =
      (state_ == ControlState::kRun || state_ == ControlState::kDegraded);
    out.faults = evaluateFaults(in, controlling);
    out.sensors_ok =
      (out.faults & (kFaultImu | kFaultJointState | kFaultAttitude | kFaultWheelState)) == 0u;
    out.command_fresh = in.command_seen && in.command_age <= limits_.command_timeout;

    const bool hard_fault = (out.faults & kHardFaultMask) != 0u;

    switch (state_) {
      case ControlState::kInit:
        // 自检阶段不因传感器缺失而报警停机，只是停在 INIT
        if (hard_fault) {
          healthy_cycles_ = 0;
          break;
        }
        if (!in.enable_request) {
          state_ = ControlState::kIdle;
          healthy_cycles_ = 0;
          break;
        }
        healthy_cycles_ = (out.sensors_ok && postureOk(in)) ? healthy_cycles_ + 1 : 0;
        if (healthy_cycles_ >= limits_.startup_cycles) {
          state_ = ControlState::kRun;
        }
        break;

      case ControlState::kIdle:
        if (hard_fault) {
          state_ = ControlState::kFault;
          healthy_cycles_ = 0;
          break;
        }
        // 躺倒时不允许站起：只是不累计自检周期，不报故障（避免 Idle↔Fault 抖动）
        healthy_cycles_ =
          (in.enable_request && out.sensors_ok && postureOk(in)) ? healthy_cycles_ + 1 : 0;
        if (healthy_cycles_ >= limits_.startup_cycles) {
          state_ = ControlState::kRun;
        }
        break;

      case ControlState::kRun:
        if (hard_fault) {
          state_ = ControlState::kFault;
          healthy_cycles_ = 0;
          break;
        }
        if (!in.enable_request) {
          state_ = ControlState::kIdle;
          healthy_cycles_ = 0;
          break;
        }
        if (!out.command_fresh) {
          state_ = ControlState::kDegraded;
        }
        break;

      case ControlState::kDegraded:
        if (hard_fault) {
          state_ = ControlState::kFault;
          healthy_cycles_ = 0;
          break;
        }
        if (!in.enable_request) {
          state_ = ControlState::kIdle;
          healthy_cycles_ = 0;
          break;
        }
        if (out.command_fresh) {
          state_ = ControlState::kRun;
        }
        break;

      case ControlState::kFault:
        // 故障消失且上层已撤掉使能 → 回待命（手动复位路径）
        if (!hard_fault && !in.enable_request) {
          state_ = ControlState::kIdle;
          healthy_cycles_ = 0;
          locked_faults_ = kFaultNone;
          break;
        }
        // 姿态越界造成的停机：姿态回正、角速度收敛后自动重新起控。
        // 只要还没到"倒伏"程度，停机期间仍保持平衡输出（见下面的 balance_enabled），
        // 否则关节一失力就会真的倒下去，没有恢复窗口。
        // 恢复判据比故障判据收紧 kRecoverFraction，留出回差，避免在门限附近反复 FAULT↔RUN。
        if (!hard_fault && (locked_faults_ & kFaultAttitude) != 0u && in.enable_request &&
          !attitudeFallen(in) &&
          std::abs(in.pitch) <= kRecoverFraction * limits_.max_pitch &&
          std::abs(in.pitch_rate) <= kRecoverFraction * limits_.max_pitch_rate)
        {
          if (++healthy_cycles_ >= kAttitudeRecoverCycles) {
            state_ = ControlState::kRun;
            healthy_cycles_ = 0;
            locked_faults_ = kFaultNone;
          }
        } else {
          healthy_cycles_ = 0;
        }
        break;
    }

    // 记录进入 FAULT 时触发的故障位：姿态自动恢复只针对"姿态越界"这一类停机
    if (state_ == ControlState::kFault && locked_faults_ == kFaultNone &&
      out.faults != kFaultNone)
    {
      locked_faults_ = out.faults;
    }

    // 姿态保护：因姿态越界停机、但还没到倒伏程度时，继续输出平衡 / VMC 力矩，
    // 让机器人自己把姿态拉回来（真正倒伏后再失力，避免在地上空转乱蹬）
    const bool attitude_recovering =
      state_ == ControlState::kFault && (locked_faults_ & kFaultAttitude) != 0u &&
      !hard_fault && in.enable_request && !attitudeFallen(in);

    faults_ = out.faults;
    out.state = state_;
    out.balance_enabled = (state_ == ControlState::kRun || state_ == ControlState::kDegraded) ||
      attitude_recovering;
    out.joints_enabled = out.balance_enabled;
    return out;
  }

private:
  /// 年龄是否可比较：NaN 表示“时间戳坏了”，必须当故障；
  /// 1e9 是“从未收到”的合法哨兵，靠后面的 timeout 比较判掉
  static bool finiteAge(double age) {return !std::isnan(age);}

  /// 站立姿态是否还允许起控（只看角度，不看角速度：
  /// 静止摆姿时接触抖动会让 pitch_rate 短时冲高，不该拦住站起）
  bool postureOk(const SafetyInputs & in) const
  {
    // NaN 参与比较恒为 false，必须显式拦掉，否则“姿态越界”永远判不出来
    return std::isfinite(in.pitch) && std::abs(in.pitch) <= limits_.max_pitch;
  }

  uint32_t evaluateFaults(const SafetyInputs & in, bool controlling) const
  {
    uint32_t faults = kFaultNone;

    if (!in.imu_ok || !finiteAge(in.imu_age) || in.imu_age > limits_.imu_timeout) {
      faults |= kFaultImu;
    }
    if (!in.joint_ok || !finiteAge(in.joint_age) || in.joint_age > limits_.joint_timeout) {
      faults |= kFaultJointState;
    }
    // 轮反馈是 LQR 的 x / x_dot 唯一来源：丢了就只剩倾角反馈，不能继续出平衡力矩
    if (!in.wheel_ok || !finiteAge(in.wheel_age) || in.wheel_age > limits_.wheel_timeout) {
      faults |= kFaultWheelState;
    }
    if (!in.lqr_ok) {
      faults |= kFaultLqr;
    }
    // 姿态越界只在出力阶段算故障：失能时机器人被动躺倒，没有可执行的安全动作
    if (controlling &&
      (!std::isfinite(in.pitch) || !std::isfinite(in.pitch_rate) ||
      std::abs(in.pitch) > limits_.max_pitch ||
      std::abs(in.pitch_rate) > limits_.max_pitch_rate))
    {
      faults |= kFaultAttitude;
    }
    if (in.estop) {
      faults |= kFaultEstop;
    }
    // 命令超时按两档处理：短期 → 降级（上面的 command_fresh），长期 → 停机
    if (in.command_seen && (!finiteAge(in.command_age) ||
      in.command_age > limits_.command_fault_timeout))
    {
      faults |= kFaultCommand;
    }
    return faults;
  }

  SafetyLimits limits_{};
  ControlState state_{ControlState::kInit};
  uint32_t faults_{kFaultNone};
  int healthy_cycles_{0};
  /// 进入 FAULT 的那一周期触发的故障位（判断能否做姿态自动恢复）
  uint32_t locked_faults_{kFaultNone};
  /// 姿态恢复后需连续健康的周期数（500 Hz 下 10 ms），太短会抖，太长会错过回正窗口
  static constexpr int kAttitudeRecoverCycles = 5;
  /// 自动恢复的姿态回差：回到门限的 75% 以内才重新起控
  static constexpr double kRecoverFraction = 0.75;
  /// 超过门限的这么多倍就认定"已经倒伏"，不再保平衡、直接失力
  static constexpr double kFallPitchFraction = 2.5;
  static constexpr double kFallRateFraction = 3.0;

  /// 是否已经倒到无法救回（NaN 也按倒伏处理）
  bool attitudeFallen(const SafetyInputs & in) const
  {
    return !std::isfinite(in.pitch) || !std::isfinite(in.pitch_rate) ||
           std::abs(in.pitch) > kFallPitchFraction * limits_.max_pitch ||
           std::abs(in.pitch_rate) > kFallRateFraction * limits_.max_pitch_rate;
  }
};

}  // namespace leg_calc

#endif  // LEG_CALC__CONTROL_STATE_HPP_
