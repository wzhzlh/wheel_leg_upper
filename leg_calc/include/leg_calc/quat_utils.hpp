#ifndef LEG_CALC__QUAT_UTILS_HPP_
#define LEG_CALC__QUAT_UTILS_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace leg_calc
{

/**
 * @brief 四元数 (w, x, y, z) 转欧拉角 (roll, pitch, yaw)，单位为 rad
 *
 * 采用 ZYX（yaw-pitch-roll）旋转顺序，与 REP-103 一致：
 *   roll  = atan2(2(wx + yz), 1 - 2(x^2 + y^2))
 *   pitch = asin(2(wy - zx))
 *   yaw   = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
 *
 * 双并腿轮足机器人绕轮轴前后倾覆对应 pitch，即 LQR 模型中的 theta。
 *
 * @param w,x,y,z 四元数分量（内部做归一化，零四元数返回全零）
 */
inline Eigen::Vector3d quaternionToRpy(double w, double x, double y, double z)
{
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (!(norm > 1e-9)) {
    return Eigen::Vector3d::Zero();
  }
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;

  const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return Eigen::Vector3d(roll, pitch, yaw);
}

}  // namespace leg_calc

#endif  // LEG_CALC__QUAT_UTILS_HPP_
