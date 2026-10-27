#include "leg_calc/foot_vmc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace leg_calc
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

/**
 * @brief 两圆交点
 *
 * 设 u 为 c0→c1 的单位向量，perp = (-u_y, u_x)（在 y 向下的坐标系里指向连线“下方”）。
 * 两个交点分别为 mid ± h·perp：p0 为 +perp 侧，p1 为 -perp 侧。
 *
 * @return 交点个数（0 = 分离/内含，1 = 相切，2 = 相交）
 */
int circleIntersection(
  const Eigen::Vector2d & c0, double r0, const Eigen::Vector2d & c1, double r1,
  Eigen::Vector2d * p0, Eigen::Vector2d * p1)
{
  const Eigen::Vector2d delta = c1 - c0;
  const double d = delta.norm();
  if (d < 1e-12 || d > r0 + r1 + 1e-9 || d < std::abs(r0 - r1) - 1e-9) {
    return 0;
  }

  const Eigen::Vector2d u = delta / d;
  const double a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
  const double h = std::sqrt(std::max(0.0, r0 * r0 - a * a));
  const Eigen::Vector2d mid = c0 + a * u;
  const Eigen::Vector2d perp(-u.y(), u.x());

  *p0 = mid + h * perp;
  *p1 = mid - h * perp;
  return h > 1e-9 ? 2 : 1;
}

}  // namespace

// ------------------------------ 几何 ------------------------------

bool FiveBarGeometry::isValid() const
{
  if (!(l1 > 0.0) || !(l2 > 0.0) || !(l3 > 0.0) || !(l4 > 0.0) || !(l5 > 0.0)) {
    return false;
  }
  // 自检位形：α = β = 90°（两摆臂竖直向下），此时 A、C 的间距必须
  // 落在 [|l2-l3|, l2+l3] 内，否则两连杆接不上（或永远重合）
  const double ac = std::hypot(l5, l4 - l1);
  return ac <= l2 + l3 + 1e-9 && ac >= std::abs(l2 - l3) - 1e-9;
}

Eigen::Vector2d FiveBarGeometry::frontElbow(double alpha) const
{
  return Eigen::Vector2d(l1 * std::cos(alpha), l1 * std::sin(alpha));
}

Eigen::Vector2d FiveBarGeometry::rearElbow(double beta) const
{
  return Eigen::Vector2d(l5 + l4 * std::cos(beta), l4 * std::sin(beta));
}

bool FiveBarGeometry::footPosition(const Eigen::Vector2d & q, Eigen::Vector2d * foot) const
{
  const Eigen::Vector2d a = frontElbow(q(0));
  const Eigen::Vector2d c = rearElbow(q(1));

  Eigen::Vector2d above;
  Eigen::Vector2d below;
  if (circleIntersection(a, l2, c, l3, &below, &above) == 0) {
    return false;
  }
  *foot = (foot_branch >= 0) ? below : above;
  return foot->allFinite();
}

bool FiveBarGeometry::jacobian(const Eigen::Vector2d & q, Eigen::Matrix2d * jac) const
{
  Eigen::Vector2d b;
  if (!footPosition(q, &b)) {
    return false;
  }
  const Eigen::Vector2d ab = b - frontElbow(q(0));
  const Eigen::Vector2d cb = b - rearElbow(q(1));

  // 两连杆长度约束对 q 求导：
  //   (B-A)·∂B/∂α = (B-A)·(∂A/∂α),  (B-C)·∂B/∂α = 0
  //   (B-A)·∂B/∂β = 0,               (B-C)·∂B/∂β = (B-C)·(∂C/∂β)
  Eigen::Matrix2d lhs;
  lhs.row(0) = ab.transpose();
  lhs.row(1) = cb.transpose();
  if (std::abs(lhs.determinant()) < 1e-9 * l2 * l3) {
    return false;  // 两连杆共线（完全伸直或折叠），雅可比奇异
  }

  const Eigen::Vector2d d_front(-l1 * std::sin(q(0)), l1 * std::cos(q(0)));
  const Eigen::Vector2d d_rear(-l4 * std::sin(q(1)), l4 * std::cos(q(1)));

  Eigen::Matrix2d rhs = Eigen::Matrix2d::Zero();
  rhs(0, 0) = ab.dot(d_front);
  rhs(1, 1) = cb.dot(d_rear);

  *jac = lhs.fullPivLu().solve(rhs);
  return jac->allFinite();
}

bool FiveBarGeometry::inverseKinematics(
  const Eigen::Vector2d & foot, Eigen::Vector2d * q, const Eigen::Vector2d * q_seed) const
{
  // 前肘：圆(O, l1) ∩ 圆(B, l2)，a[0] 在 O→B 左侧，a[1] 右侧
  Eigen::Vector2d a[2];
  if (circleIntersection(hipNear(), l1, foot, l2, &a[0], &a[1]) == 0) {
    return false;
  }
  // 后肘：圆(D, l4) ∩ 圆(B, l3)，c[0] 在 D→B 左侧，c[1] 右侧
  Eigen::Vector2d c[2];
  if (circleIntersection(hipFar(), l4, foot, l3, &c[0], &c[1]) == 0) {
    return false;
  }

  auto to_joint = [this](const Eigen::Vector2d & front, const Eigen::Vector2d & rear) {
      return Eigen::Vector2d(
        std::atan2(front.y(), front.x()), std::atan2(rear.y(), rear.x() - l5));
    };

  if (q_seed == nullptr) {
    // 图中装配：A 在 O→B 左侧，C 在 D→B 右侧
    const Eigen::Vector2d & front = (front_branch >= 0) ? a[0] : a[1];
    const Eigen::Vector2d & rear = (rear_branch >= 0) ? c[1] : c[0];
    *q = to_joint(front, rear);
    return q->allFinite();
  }

  // 有种子时取 4 组装配里离种子最近的一组（角度按 2π 归一），保证连续
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      const Eigen::Vector2d candidate = to_joint(a[i], c[j]);
      const Eigen::Vector2d diff(
        std::remainder(candidate(0) - (*q_seed)(0), 2.0 * kPi),
        std::remainder(candidate(1) - (*q_seed)(1), 2.0 * kPi));
      const double cost = diff.norm();
      if (cost < best) {
        best = cost;
        *q = candidate;
      }
    }
  }
  return q->allFinite();
}

bool FiveBarGeometry::jointVelocity(
  const Eigen::Vector2d & q, const Eigen::Vector2d & foot_velocity,
  Eigen::Vector2d * q_dot) const
{
  Eigen::Matrix2d jac;
  if (!jacobian(q, &jac)) {
    return false;
  }
  if (std::abs(jac.determinant()) < 1e-12) {
    return false;
  }
  *q_dot = jac.fullPivLu().solve(foot_velocity);
  return q_dot->allFinite();
}

bool FiveBarGeometry::jointTorque(
  const Eigen::Vector2d & q, const Eigen::Vector2d & foot_force,
  Eigen::Vector2d * torque) const
{
  Eigen::Matrix2d jac;
  if (!jacobian(q, &jac)) {
    return false;
  }
  *torque = jac.transpose() * foot_force;
  return torque->allFinite();
}

// ------------------------------ VMC ------------------------------

Eigen::Vector2d springDamperForce(
  const SpringDamperParams & params,
  const Eigen::Vector2d & target, const Eigen::Vector2d & target_velocity,
  const Eigen::Vector2d & current, const Eigen::Vector2d & current_velocity)
{
  Eigen::Vector2d force =
    params.kp.cwiseProduct(target - current) +
    params.kd.cwiseProduct(target_velocity - current_velocity);

  for (int i = 0; i < 2; ++i) {
    const double limit = params.force_max(i);
    if (limit > 0.0) {
      force(i) = std::clamp(force(i), -limit, limit);
    }
  }
  return force;
}

bool computeVmcLeg(
  const FiveBarGeometry & geometry, const SpringDamperParams & params,
  const Eigen::Vector2d & q, const Eigen::Vector2d & q_dot,
  const Eigen::Vector2d & target, const Eigen::Vector2d & target_velocity,
  VmcLegState * out)
{
  Eigen::Vector2d foot;
  Eigen::Matrix2d jac;
  if (!geometry.footPosition(q, &foot) || !geometry.jacobian(q, &jac)) {
    return false;
  }

  const Eigen::Vector2d foot_velocity = jac * q_dot;         // ẋ = J q̇
  // F = Kp·(x_d − x) + Kd·(ẋ_d − ẋ)
  const Eigen::Vector2d force =
    springDamperForce(params, target, target_velocity, foot, foot_velocity);
  const Eigen::Vector2d torque = jac.transpose() * force;     // τ = Jᵀ F

  if (!force.allFinite() || !torque.allFinite()) {
    return false;
  }

  out->joint_position = q;
  out->joint_velocity = q_dot;
  out->foot_position = foot;
  out->foot_velocity = foot_velocity;
  out->foot_force = force;
  out->joint_torque = torque;
  out->jacobian = jac;
  return true;
}

}  // namespace leg_calc
