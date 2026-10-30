#ifndef LEG_CALC__ROBOT_COMMAND_HPP_
#define LEG_CALC__ROBOT_COMMAND_HPP_

// 外部上层指令源 → leg_control 的命令帧格式定义
//
// 话题：/robot_command，类型 std_msgs/msg/Float64MultiArray
// 字段顺序固定，发布端与接收端都引用这里，避免两边各写一份下标。
//
//   [0] v_des      目标线速度 [m/s]          （正 = 前进）
//   [1] yaw_des    目标偏航角速度 [rad/s]    （正 = 左转）
//   [2] com_height 目标质心高度 [m]，<=0 表示“用控制器默认值”
//   [3] pitch_des  目标车身 pitch [rad]      （LQR 参考值）
//   [4] enable     >0.5 请求使能
//   [5] estop      >0.5 请求急停
//
// 注意：命令帧只描述“上层意图”，不包含任何控制增益或电机指令；
// 500 Hz 控制环读到新命令后立即采用，命令超时由状态机处理（降级 → 停机）。

#include <cstddef>
#include <vector>

namespace leg_calc
{

/// /robot_command 的字段下标
enum CommandIndex : size_t
{
  kCmdLinearVelocity = 0,
  kCmdYawRate = 1,
  kCmdComHeight = 2,
  kCmdPitchTarget = 3,
  kCmdEnable = 4,
  kCmdEstop = 5,
  kCmdCount = 6,
};

/// 一条上层命令（单位见 CommandIndex 注释）
struct RobotCommand
{
  double linear_velocity{0.0};
  double yaw_rate{0.0};
  double com_height{-1.0};  ///< 目标质心高度；<=0 表示使用控制器参数里的默认值
  double pitch_target{0.0};
  bool enable{false};
  bool estop{false};

  /// 从话题数据还原；长度不足时返回默认命令（不使能）
  static RobotCommand fromArray(const std::vector<double> & data)
  {
    RobotCommand cmd;
    if (data.size() < static_cast<size_t>(kCmdCount)) {
      return cmd;
    }
    cmd.linear_velocity = data[kCmdLinearVelocity];
    cmd.yaw_rate = data[kCmdYawRate];
    cmd.com_height = data[kCmdComHeight];
    cmd.pitch_target = data[kCmdPitchTarget];
    cmd.enable = data[kCmdEnable] > 0.5;
    cmd.estop = data[kCmdEstop] > 0.5;
    return cmd;
  }

  /// 打包成话题数据
  std::vector<double> toArray() const
  {
    std::vector<double> data(static_cast<size_t>(kCmdCount), 0.0);
    data[kCmdLinearVelocity] = linear_velocity;
    data[kCmdYawRate] = yaw_rate;
    data[kCmdComHeight] = com_height;
    data[kCmdPitchTarget] = pitch_target;
    data[kCmdEnable] = enable ? 1.0 : 0.0;
    data[kCmdEstop] = estop ? 1.0 : 0.0;
    return data;
  }
};

}  // namespace leg_calc

#endif  // LEG_CALC__ROBOT_COMMAND_HPP_
