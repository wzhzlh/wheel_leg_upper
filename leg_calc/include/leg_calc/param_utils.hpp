#ifndef LEG_CALC__PARAM_UTILS_HPP_
#define LEG_CALC__PARAM_UTILS_HPP_

// 三个节点（实时控制器 / 离线演示 / 增益表工具）共用的参数声明与读取。
//
// 用法：
//   declareModelParams(*this);            // 声明物理参数 + input_mode
//   declareLqrWeights(*this);             // 声明 q_diag / r_weight
//   ...
//   const ModelParams params = readModelParams(*this);
//
// 参数回调场景下，新值尚未写入节点，需要传 pending（本次 set 的参数），
// 否则 get_parameter 读到的仍是旧值：
//   readModelParams(*this, &pending);

#include <Eigen/Dense>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "leg_calc/lqr.hpp"

namespace leg_calc
{

/// 参数回调中“待提交”的参数集合
using ParamMap = std::map<std::string, rclcpp::Parameter>;

namespace detail
{

/// 优先取 pending 中的新值，其次取节点上已生效的值
inline rclcpp::Parameter lookupParam(
  rclcpp::Node & node, const ParamMap * pending, const std::string & name)
{
  if (pending != nullptr) {
    const auto it = pending->find(name);
    if (it != pending->end()) {
      return it->second;
    }
  }
  return node.get_parameter(name);
}

}  // namespace detail

/// 声明模型参数：cart_mass / pendulum_mass / com_length / com_inertia /
/// gravity / wheel_radius / wheel_inertia / input_mode
inline void declareModelParams(rclcpp::Node & node)
{
  const ModelParams defaults;
  node.declare_parameter("cart_mass", defaults.cart_mass);
  node.declare_parameter("pendulum_mass", defaults.pendulum_mass);
  node.declare_parameter("com_length", defaults.com_length);
  node.declare_parameter("com_inertia", defaults.com_inertia);
  node.declare_parameter("gravity", defaults.gravity);
  node.declare_parameter("wheel_radius", defaults.wheel_radius);
  node.declare_parameter("wheel_inertia", defaults.wheel_inertia);
  node.declare_parameter("input_mode", std::string("wheel_torque"));
}

/// 读取模型参数（pending 非空时优先使用其中的新值）
inline ModelParams readModelParams(rclcpp::Node & node, const ParamMap * pending = nullptr)
{
  ModelParams params;
  params.cart_mass = detail::lookupParam(node, pending, "cart_mass").as_double();
  params.pendulum_mass = detail::lookupParam(node, pending, "pendulum_mass").as_double();
  params.com_length = detail::lookupParam(node, pending, "com_length").as_double();
  params.com_inertia = detail::lookupParam(node, pending, "com_inertia").as_double();
  params.gravity = detail::lookupParam(node, pending, "gravity").as_double();
  params.wheel_radius = detail::lookupParam(node, pending, "wheel_radius").as_double();
  params.wheel_inertia = detail::lookupParam(node, pending, "wheel_inertia").as_double();
  params.input_mode =
    (detail::lookupParam(node, pending, "input_mode").as_string() == "force") ?
    InputMode::kForce : InputMode::kWheelTorque;
  return params;
}

/// 声明 LQR 权重：q_diag(4 个) / r_weight
inline void declareLqrWeights(
  rclcpp::Node & node,
  const std::vector<double> & q_diag_default = {10.0, 500.0, 1.0, 1.0},
  double r_weight_default = 1.0)
{
  node.declare_parameter("q_diag", q_diag_default);
  node.declare_parameter("r_weight", r_weight_default);
}

/// 读取 LQR 权重，直接组装成 Q/R 矩阵
inline void readLqrWeights(
  rclcpp::Node & node, Eigen::MatrixXd * Q, Eigen::MatrixXd * R,
  const ParamMap * pending = nullptr)
{
  const std::vector<double> q_diag =
    detail::lookupParam(node, pending, "q_diag").as_double_array();
  if (q_diag.size() != static_cast<size_t>(kStateDim)) {
    throw std::invalid_argument("q_diag 必须包含 4 个元素");
  }
  const double r_weight = detail::lookupParam(node, pending, "r_weight").as_double();

  *Q = Eigen::VectorXd::Map(
    q_diag.data(), static_cast<Eigen::Index>(q_diag.size())).asDiagonal();
  *R = Eigen::MatrixXd::Constant(1, 1, r_weight);
}

}  // namespace leg_calc

#endif  // LEG_CALC__PARAM_UTILS_HPP_
