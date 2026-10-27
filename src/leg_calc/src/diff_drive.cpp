#include "leg_calc/diff_drive.hpp"

#include <algorithm>
#include <cmath>

namespace leg_calc
{
namespace
{

/// 逐轴限幅：limit <= 0 表示不限
double clampAxis(double value, double limit, bool * saturated)
{
  if (limit > 0.0 && std::abs(value) > limit) {
    if (saturated != nullptr) {
      *saturated = true;
    }
    return std::clamp(value, -limit, limit);
  }
  return value;
}

}  // namespace

DiffDriveState bodyFromWheels(
  const DiffDriveParams & params, double left_speed, double right_speed)
{
  DiffDriveState state;
  if (!params.isValid()) {
    return state;
  }

  const double r = params.wheel_radius;
  state.left_speed = left_speed;
  state.right_speed = right_speed;
  state.left_linear = left_speed * r;
  state.right_linear = right_speed * r;
  state.linear = 0.5 * (state.left_linear + state.right_linear);
  state.angular = (right_speed - left_speed) * r / params.wheel_separation;
  return state;
}

DiffDriveState wheelsFromBody(
  const DiffDriveParams & params, double linear, double angular)
{
  DiffDriveState state;
  if (!params.isValid()) {
    return state;
  }

  // 1. 车体量独立限幅
  bool saturated = false;
  linear = clampAxis(linear, params.max_linear, &saturated);
  angular = clampAxis(angular, params.max_angular, &saturated);
  state.linear = linear;
  state.angular = angular;

  // 2. 分解到左右轮：ω_L = (v − L/2·ω)/r，ω_R = (v + L/2·ω)/r
  const double r = params.wheel_radius;
  const double half = 0.5 * params.wheel_separation;
  double left = (linear - half * angular) / r;
  double right = (linear + half * angular) / r;

  // 3. 轮速限幅：按同一比例缩放，保持转向半径
  const double peak = std::max(std::abs(left), std::abs(right));
  if (params.max_wheel_speed > 0.0 && peak > params.max_wheel_speed) {
    const double scale = params.max_wheel_speed / peak;
    left *= scale;
    right *= scale;
    // 实际能达到的车体速度也一起缩放，便于上层知道指令被削弱了多少
    state.linear *= scale;
    state.angular *= scale;
    saturated = true;
  }

  state.left_speed = left;
  state.right_speed = right;
  state.left_linear = left * r;
  state.right_linear = right * r;
  state.saturated = saturated;
  return state;
}

}  // namespace leg_calc
