#ifndef LEG_CALC__LEG_CONTROL_TYPES_HPP_
#define LEG_CALC__LEG_CONTROL_TYPES_HPP_

// 统一控制链路的三个纯数据结构（不依赖 ROS，可在单测里直接构造）
//
//   ControllerConfig : 一组在周期边界生效的控制参数
//   StateSnapshot    : 一个控制周期取到的“一致快照”（各通道数据 + 年龄）
//   ControlOutput    : 该周期的控制结果（状态机状态 + 关节力矩 + 轮速目标）
//
// 输入输出与计算分离：ROS 侧的字段解析在 leg_control_io，控制算法在 leg_control_core。

#include <Eigen/Dense>

#include <array>
#include <string>
#include <vector>

#include "leg_calc/control_state.hpp"
#include "leg_calc/foot_vmc.hpp"
#include "leg_calc/lqr.hpp"
#include "leg_calc/robot_command.hpp"
#include "leg_calc/wheel_leg_model.hpp"

namespace leg_calc
{

/// 一组只在周期边界生效的控制配置
struct ControllerConfig
{
  ModelParams model;
  Eigen::MatrixXd Q{Eigen::Matrix4d::Identity()};
  Eigen::MatrixXd R{Eigen::MatrixXd::Identity(1, 1)};
  double u_max{5.0};

  std::string theta_axis{"pitch"};
  int axis_index{1};
  double theta_offset{0.0};
  GainMode gain_mode{GainMode::kQuadratic};
  Eigen::Matrix<double, 4, 3> k_coeffs{Eigen::Matrix<double, 4, 3>::Zero()};
  double h_min{0.10};
  double h_max{0.60};

  LegGeometry leg_geometry;
  bool use_measured_height{true};
  double height_timeout{0.50};

  FiveBarGeometry fivebar;
  SpringDamperParams vmc;
  double foot_target_x{0.05};
  double com_height_default{0.40};
  double com_height_min{0.25};
  double com_height_max{0.50};
  double com_height_rate{0.30};  ///< 目标质心高度斜坡 [m/s]
  double joint_torque_max{8.0};
  std::array<double, 4> joint_torque_sign{{1.0, 1.0, 1.0, 1.0}};

  /// 关节角 → 五杆角标定：α/β[i] = joint_sign[i]·q[i] + joint_offset[i]
  /// 顺序同 leg_joint_names：[α_L, β_L, α_R, β_R]。
  /// 曲柄在 URDF/MJCF 里的零位并不指向 +x（左腿 123.5°/34.3°，右腿 119.9°/55.6°），
  /// 右腿还因为绕 x 轴转了 180° 而反向，所以这 4 组数必须标定（见 teach.md 12.7）。
  std::array<double, 4> joint_sign{{1.0, 1.0, 1.0, 1.0}};
  std::array<double, 4> joint_offset{{0.0, 0.0, 0.0, 0.0}};

  double wheel_separation{0.20};
  double wheel_radius{0.065};
  double max_wheel_omega{20.0};
  double max_balance_omega{8.0};
  double max_drive_omega{12.0};
  double balance_torque_to_omega{1.0};  ///< [rad/s per N·m]，按实机标定
  double balance_leak_rate{6.0};        ///< 平衡轮速指令一阶低通系数 [1/s]，越大跟随越快

  SafetyLimits safety;
  double control_rate{500.0};
  double max_linear_velocity{1.0};
  double max_yaw_rate{3.0};

  std::vector<std::string> leg_joint_names;
  std::vector<std::string> wheel_joint_names;
};

/// 一个控制周期的一致快照；age 单位 s，1e9 表示“从未收到”
struct StateSnapshot
{
  bool imu_ok{false};
  double imu_age{1e9};
  bool joint_ok{false};
  double joint_age{1e9};
  bool wheel_ok{false};
  double wheel_age{1e9};  ///< 距最近一次有效轮反馈的时间 [s]

  double pitch{0.0};
  double pitch_rate{0.0};
  double yaw{0.0};        ///< IMU 偏航角 [rad]（用于 2D 里程计朝向）
  double yaw_rate{0.0};   ///< IMU 偏航角速度 [rad/s]（绕 z 轴）
  double x{0.0};
  double x_dot{0.0};

  std::array<double, 4> joint_position{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> joint_velocity{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 2> wheel_omega{{0.0, 0.0}};

  bool command_seen{false};
  double command_age{1e9};
  RobotCommand command{};
};

/// 一个控制周期的输出
struct ControlOutput
{
  ControlState state{ControlState::kInit};
  uint32_t faults{kFaultNone};

  std::array<double, 4> joint_torque{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 2> wheel_omega_ref{{0.0, 0.0}};

  double u{0.0};            ///< Pitch LQR 输出的轮轴平衡力矩 [N·m]
  double com_height{0.30};  ///< 本周期实测质心高度（五杆 FK 反算）[m]
  double com_height_cmd{0.40};  ///< 目标质心高度（斜坡后）[m]

  bool balance_enabled{false};  ///< 本周期是否在出平衡/VMC 力矩
  bool joints_enabled{false};   ///< 本周期是否在出关节力矩
  bool command_fresh{false};    ///< 上层命令是否新鲜（决定是否跟踪速度）
};

}  // namespace leg_calc

#endif  // LEG_CALC__LEG_CONTROL_TYPES_HPP_
